# 실내 시뮬레이션 장애물 자동 재개 — 2026-11-22

`indoor_demo.launch.py`는 이제 `auto_resume:=true`가 기본이다. 최초 arm/start는 여전히 필요하다.
실차 인터페이스나 공식 `mitt_navigation`은 변경하지 않았다.

## 동작

1. 기존 안전 게이트의 `obstacle_stop`, 또는 장애물 감속 중 Nav2 추종 실패로 분류된 정지를 기다린다.
2. 센서 timestamp·수신 상태가 정상이고 실제 모의 속도가 0.01m/s 미만이어야 한다.
3. 직진 기준 전방 2m에 차체 크기와 여유를 더한 영역이 **3초 연속 비어 있어야** 한다.
4. 허가 상태로 전환한 뒤 저장한 목표까지 Smac Hybrid로 새 경로를 계획하고 RPP로 추종한다.

장애물이 다시 관측되면 3초 확인 시간을 초기화한다. 센서 누락·비정상 값·시간 이상·명령 오류·운영자 disarm은
자동으로 해제하지 않는다. 대기 중 센서 timeout도 수동 재허가를 요구한다.
단순 계획 실패나 분류되지 않은 추종 실패를 자동 재시도하지 않는다.

이 기능은 장애물의 속도나 미래 궤적을 예측하지 않는다. 좁은 곡선 통로에서는 직진 여유 검사 때문에 계속 기다릴 수 있다.
기존 늦은 측면 진입의 한계도 남아 있다. 모의 주행 전용이며 실차 안전 기능을 대신하지 않는다.

## 기존 컨테이너에서 적용

호스트의 커밋된 코드를 동기화한다.

```bash
cd ~/mrider-team
bash tools/ros2/sync_existing_container.sh ros2_humble
docker exec -it ros2_humble bash
```

기존 demo는 그 demo를 실행한 터미널에서 Ctrl+C로 종료한다. 컨테이너 안에서 재빌드하고 시작한다.

```bash
cd /root/mrider-team
source ros2_ws/setup_env.sh
export ROS_DOMAIN_ID=191
cd ros2_ws
colcon build --build-base build-docker --install-base install-docker --packages-up-to mrider_navigation_sim --symlink-install
source install-docker/setup.bash
ros2 launch mrider_navigation_sim indoor_demo.launch.py
```

다른 터미널에서도 같은 컨테이너·setup·domain을 준비한 뒤 최초 주행을 요청한다.

```bash
ros2 service call /mrider_sim/arm std_srvs/srv/Trigger '{}'
ros2 service call /mrider_sim/start std_srvs/srv/Trigger '{}'
```

이후 횡단 장애물 정지는 조건 충족 시 자동 재개한다. 이전 수동 재허가 방식은
`ros2 launch mrider_navigation_sim indoor_demo.launch.py auto_resume:=false`로 유지할 수 있다.
범용 `navigation_sim.launch.py`는 기존 회귀 시험을 위해 기본값이 false다.

## 검증

수동 demo를 종료하고 컨테이너의 `ros2_ws`에서:

```bash
colcon test --build-base build-docker --install-base install-docker --packages-select mrider_navigation_sim --ctest-args -j1 --output-on-failure
colcon test-result --test-result-base build-docker/mrider_navigation_sim --verbose
```

기존 `ros2_humble`에서 Navigation 개별 시험 54개가 통과했다(CTest wrapper 포함 63개, 실패 0).
이후 추가한 보호 시험 2개도 별도로 통과하여 이번 변경의 개별 검사는 총 56개다.
자동 횡단 재개 시험은 목표 오차 **0.172290m**, 최소 이격 **0.537358m**, 충돌 false,
`manual_rearm_required=false`였다. 기존 수동 재허가 시나리오도 그대로 통과했다.
센서 timeout 후 입력 복구, 연속 여유 시간 초기화, 운영자 disarm의 자동 재개 금지도 검사했다.

[Navigation 로그](results/auto-resume-2026-11-22/navigation-tests.log),
[보호 시험](results/auto-resume-2026-11-22/protection-tests.log),
[시나리오 측정값](results/auto-resume-2026-11-22/scenarios.json),
[XML](results/auto-resume-2026-11-22/xml/)을 보관했다.
이번에는 수정한 팀 Navigation 패키지를 검증했으며 공식 원본 lint 실패는 별도 기존 기록을 따른다.
