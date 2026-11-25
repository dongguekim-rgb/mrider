# 건물 내부·이동 장애물 시뮬레이션

2026-11-22 업데이트: 실내 demo는 [장애물 자동 재개](automatic-obstacle-resume.md)가 기본이다. 아래 수동 재허가 절차는 `auto_resume:=false` 또는 센서·명령 오류로 정지한 경우에 사용한다.

platinum의 기존 `ros2_humble`에서 실행하는 현재 절차는 [기존 컨테이너 설명서](existing-humble-container.md)를 따른다.

ROS2 Humble의 기존 방향 A Navigation을 사무실형 실내로 확장한 환경이다.
학습 알고리즘 없이 Smac Hybrid 계획 → RPP 추종 → 모의 안전 게이트를 사용한다.

## 공간과 동작

- 14 × 10 m 외벽, 폭 5 m 중앙 복도, 사무실·회의실·창고·리셉션·라운지 등 6개 공간.
- 복도 양쪽의 1.5 / 2 / 1.5 m 출입구, 책상·수납장·복도 가장자리 운반 카트.
- 시작 후륜 축 `(0, 0, 0)`, 기본 목표 `(8, 0, 0)`; 단위 m/rad.
- 회색 벽, 갈색 가구, 주황색 이동 장애물, 방 이름을 RViz2에 표시한다.
- `indoor`: 정적 실내 지도. `indoor_dynamic`: 동일 지도에 0.5 × 0.5 m 사각 충돌 영역의 보행자 모형 2개를 추가한다.

횡단 모형은 `(6, -2.1)`에서 기다리다가 차량 후륜 축 x가 2.5 m 이상이 되면
0.6 m/s로 복도 중앙까지 이동한다. 3초 기다린 뒤 반대편 출입구를 통해 `(6, 3.3)`까지 이동해 멈춘다.
다른 모형은 Office B 안에서 주기적으로 움직인다. 차량이 정지해도 모형의 시간은 진행된다.
횡단은 launch마다 한 번만 발생한다. 반복하려면 launch를 종료하고 다시 실행한다.

이동 물체는 **정적 `/map`에는 없고 `/scan`, Nav2 장애물 costmap, 차체 충돌 판정에는 존재**한다.
따라서 마커 애니메이션만 추가한 구성이 아니다. 통로가 위험하면 정지 latch를 유지하며,
기본 자동 재개 모드에서는 여유 확인 후 재계획한다. 수동 모드에서는 재허가를 기다린다. Nav2가 먼저 추종을 중단한 경우 임무는 `FAILED`,
게이트가 먼저 정지시킨 경우 `STOPPED`가 될 수 있다. 두 경우 모두 안전 상태 `STOPPED`와 속도 0을 확인한다.

## 다른 PC에서 설치·빌드·실행

Ubuntu 22.04 / ROS2 Humble 설치는 [새 PC 설명서](new-pc-setup.md)를 따른다.
**호스트에 `/opt/ros/humble/setup.bash`가 존재해야 한다.** Docker 안에만 설치한 Humble은
호스트에서 source할 수 없다. setup 파일이 없으면 [환경 확인·복구 절차](new-pc-setup.md)를 먼저 따른다.
ROS1 catkin 작업 공간 대신 `ros2_ws`에서 colcon으로 빌드한다.

```bash
source /opt/ros/humble/setup.bash
sudo apt-get install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rviz2
cd ~/mrider-team/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
export ROS_DOMAIN_ID=187
ros2 launch mrider_navigation_sim indoor_demo.launch.py
```

정적 실내만 보려면 `scenario:=indoor`, 화면이 없는 서버에서는 `rviz:=false`를 추가한다.
동일 도메인에서 다른 차량 demo 또는 자동 검증을 동시에 실행하지 않는다.
Nav2가 `Managed nodes are active`를 출력한 후 별도 터미널에서:

```bash
source /opt/ros/humble/setup.bash
source ~/mrider-team/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=187
ros2 service call /mrider_sim/arm std_srvs/srv/Trigger '{}'
ros2 service call /mrider_sim/start std_srvs/srv/Trigger '{}'
```

서비스의 `success: true`를 확인한다. `nav2_not_ready`이면 초기화 완료를 기다린다.
횡단 후 정지한 차량을 다시 출발시키려면 보행자가 반대편 방으로 들어간 것을 확인하고
위 arm/start 두 명령을 다시 실행한다. 목표에 도착하면 속도가 0이 된다.
안전 상태는 `ros2 topic echo /mrider_sim/safety_state`, 충돌 여부는
`ros2 topic echo /mrider_sim/collision`으로 관찰한다.
RViz2의 Goal Pose로 다른 목표도 지정할 수 있지만 모든 방 진입과 좁은 통로의 완주는 검증하지 않았다.

## 자동 검증

수동 demo를 종료한 다음, 작업 공간에서 실행한다. 시험이 각 월드를 직접 실행한다.

```bash
cd ~/mrider-team/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export MRIDER_SIM_REPORT_DIR="$PWD/log/indoor"
colcon test --packages-select mrider_navigation_sim --ctest-args -R indoor -j1 --output-on-failure
colcon test-result --verbose
```

전체 회귀 검증은 저장소 루트에서 `bash tools/ros2/verify.sh`를 실행한다.
이미 켜진 새 실내 launch를 검사하려면 수동 arm/start 대신
`ros2 run mrider_navigation_sim run_scenario indoor_auto`을 사용한다.
수동 모드(`auto_resume:=false`)에는 `run_scenario indoor_dynamic`, 정적 실내 launch에는 `run_scenario indoor`를 사용한다. 실행한 월드와 검사 이름을 반드시 맞춘다.

합격 기준:

- 정적 실내: 목표 위치 오차 ≤ 0.23 m, 방향 오차 ≤ 0.25 rad, 충돌 없음.
- 동적 실내: 횡단 중 정지, 정지 후에도 보행자 이동, 통로가 비어도 자동 재출발 없음,
  명시적 arm/start 뒤 목표 오차 ≤ 0.23 m, 전 과정 충돌 없음.
- 두 경우 명령 속도 ≤ 0.5 m/s, 조향 크기 ≤ 0.3937 rad, 차량과 물체의 최소 거리 > 0.
- 기하 시험: 시작/목표 공간, 출입구 광선 통과, 이동 물체의 벽·가구 비충돌, 시간 입력 검증.

결과 JSON의 `trajectory`는 모의 후륜 축 x/y/속도이며, `pedestrian_trajectory`는 횡단 모형 x/y이다.
실제 실행 결과는 [실내 검증 기록](indoor-simulation-validation.md)에 남긴다.

## 한계와 변경 위치

이 지도는 사용자 요청에 따라 만든 **가상 사무실 배치**이며 실제 건물 도면이 아니다.
차량 회전 반경을 고려해 복도를 넓게 잡았다. 5 m 복도에서의 통과가 실제 좁은 복도 통과를 보장하지 않는다.
ROS2 2D 광선·자전거 모델을 사용하며, RViz 높이는 표시용이다. Gazebo 물리·카메라 영상·
LiDAR 반사/유리/노이즈·SLAM/EKF·보행자 행동 예측·실물 A2M4-R2·Teensy는 이번 검증 범위가 아니다.
특히 늦은 측면 진입은 정지만으로 접촉을 피하지 못할 수 있다. 초기 개발 시험에서 확인한
조건과 한계는 [검증 기록](indoor-simulation-validation.md)에 명시한다.
벽·가구는 `mrider_navigation_sim/indoor.py`, 가상 차량 설정은 `config/simulation.yaml`에 있다.
차량 치수 변경 시 Nav2 footprint와 회전 반경도 함께 맞춰야 한다.
모의 명령을 차량에 연결하지 않는다.


## 큰 실내 맵 추가

[60×40 m 다중 방 환경](large-office-simulation.md)은 논문 참고 설계와 오프라인 기하 검증 단계입니다. 기존 작은 맵의 주행 성공과 구분하며, 현재 시작 위치에서 카메라 미확인 판정이 발생합니다.
