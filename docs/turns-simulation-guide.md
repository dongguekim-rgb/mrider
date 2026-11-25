# 좌·우 조향과 보행자 자동 재개 통합 시나리오

> 범위 정정: 이 문서의 기존 구조도·주행 결과는 정답 지도/위치를 사용하는 기준 시험입니다. 사용자 요구인 카메라·LiDAR만의 주행은 [센서 전용 실험 경로](sensor-only-navigation.md)에서 별도로 구현·검증하며, 기존 결과를 그 성공 근거로 사용하지 않습니다.

기존 `indoor_demo.launch.py`의 직선 횡단 시험을 보완하는 별도 시나리오다.
복도 중앙의 정적 장애물을 우회하며 양방향으로 조향한 뒤, 보행자 횡단에서 정지하고
안전 조건 충족 후 저장 목표로 재계획·재출발한다. 목표는 `(20, 0)` m다.
이는 좌·우 굴곡 우회 검증이며 교차로에서 각각 90도 좌회전·우회전을 하는 시험은 아니다.

## 실행

기존 수동 demo를 종료하고 두 터미널에서 다음 환경을 설정한다.

```bash
docker exec -it ros2_humble bash
cd /root/mrider-team
source ros2_ws/setup_env.sh
source ros2_ws/install-docker/setup.bash
export ROS_DOMAIN_ID=191
```

새 코드 반영 후 최초 한 번은 컨테이너에서 빌드한다.

```bash
cd /root/mrider-team/ros2_ws
colcon --log-base log-docker build --build-base build-docker --install-base install-docker \
  --symlink-install --packages-select mrider_navigation_sim --cmake-args -DBUILD_TESTING=ON
source install-docker/setup.bash
```

터미널 1:

```bash
ros2 launch mrider_navigation_sim turns_demo.launch.py
```

터미널 2 (검사기가 arm/start를 수행하므로 수동 출발하지 않는다):

```bash
export MRIDER_SIM_REPORT_DIR=/root/mrider-team/docs/build/evidence/turns-manual
ros2 run mrider_navigation_sim run_scenario indoor_turns
```

## 합격 조건

- 주행 중 좌조향 `> +0.1 rad`, 우조향 `< -0.1 rad` 모두 관측.
- odometry yaw 변화량을 누적하여 좌·우 방향 각각 20도 초과 확인.
- 중심선으로부터 실제 횡방향 이동량 0.8 m 초과: 명령만 바뀌고 차량이 직진하는 경우 제외.
- 보행자 횡단에 따른 정지와 안전 조건 충족 후 자동 재개.
- 충돌 없음, 최소 차체 이격 거리 양수, 최종 속도 0.01 m/s 미만.
- 목표 위치 오차 0.23 m 이하, 속도·조향 제한 준수.

횡단 장애물의 x 위치는 12 m, 차량 x=8.5 m에서 이동을 시작한다.
지도는 x 방향으로 확장한 가상 실내 환경이며 실측 건물·차량을 의미하지 않는다.
안전 게이트와 Nav2 제어 설정은 기존 값을 유지한다.

## 자동 시험

수동 demo를 종료한 후 컨테이너의 `ros2_ws`에서 실행한다.

```bash
export MRIDER_SIM_REPORT_DIR=/root/mrider-team/docs/build/evidence/turns-test
colcon --log-base log-docker test --build-base build-docker --install-base install-docker \
  --packages-select mrider_navigation_sim --event-handlers console_direct+ \
  --ctest-args -R 'test_turns_launch|safety|indoor_geometry' --output-on-failure
```

새 launch 시험은 domain 211을 사용한다. 기존 실차나 다른 시험을 이 domain으로 실행하지 않는다.
JSON은 목표 오차·양방향 조향·누적 회전량·최대 횡이동·최소 이격 거리·충돌 여부를 기록한다.
이 시험은 카메라·SLAM·실제 센서·Teensy 검증을 포함하지 않는다.

## 2026-11-23 Docker 실행 결과

`ros2_humble`의 `/root/mrider-team`에서 패키지 재빌드 성공.
안전 단위 시험 35개, 실내 기하 시험 7개, 새 주행·정상 종료 시험 2개: **총 44개 통과**.
이는 이번에 선택 실행한 시험의 결과이며 저장소 전체 CI 결과가 아니다.

| 측정 | 결과 |
|---|---:|
| 목표 위치 오차 | 0.173183 m |
| 최소 이격 거리 | 0.532012 m |
| 최대 좌조향 | 0.115473 rad |
| 최대 우조향 (음수) | -0.284839 rad |
| 좌방향 누적 yaw 변화 | 34.512° |
| 우방향 누적 yaw 변화 | 32.314° |
| 최대 횡이동 | 1.400041 m |
| 충돌 | 없음 |
| 장애물 정지 후 재개 | 자동 |

누적 yaw 변화는 각 방향으로 움직인 양의 합이며, 단일 교차로 회전각이나 최종 방향 오차가 아니다.
Nav2가 횡단 장애물을 감지해 추종을 중단한 뒤 안전 조건에 따라 재계획한 로그도 보존했다.

[측정 JSON](build/evidence/turns-2026-11-23/indoor_turns.json) ·
[주행 및 종료 로그](build/evidence/turns-2026-11-23/launch-test.log) ·
[주행 시험 XML](build/evidence/turns-2026-11-23/test_test_turns_launch.py.xunit.xml)

## 동영상

[한글 MP4와 원본 기록](build/evidence/turns-video-2026-11-23/README.md).
영상은 별도 재실행의 시간별 ROS 기록을 시각화한 것으로 RViz 화면 녹화가 아니다.
