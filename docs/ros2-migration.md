# ROS2 이식 범위 — 2026-11-17

사용자 요구에 따라 현행 소프트웨어 기반을 Ubuntu 22.04 / ROS2 Humble로 변경했다.
Teensy 펌웨어 개발/교체 및 실차 구동은 이번 호스트 소프트웨어 이식 범위와 구분한다.

| 이전 | 현행 |
|---|---|
| ros1_ws / Catkin | ros2_ws / ament + colcon |
| rospy 노드·XML launch | rclpy 노드·Python launch |
| rospy Time.to_sec | builtin_interfaces Time sec/nanosec |
| 후륜축 rear_axle | 지면 후륜축 중심 base_link |
| 기본 모의 제어 30Hz | 50Hz (실제 DBW 50Hz 이상 보장은 별도 측정) |
| rostest | launch_testing + pytest |
| Gazebo ROS1 ModelState | ROS2 SpawnEntity/SetEntityState 서비스 |
| ROS1 RViz | RViz2 |

최초 이식 커밋에서는 활성 기능(학습 없는 제어·모의 차량·시각화)을 옮겼다. 당시 미구현이던
SLAM/EKF/Nav2·collision gate·실차 bringup까지 완료한 것으로 해석하지 않는다.
카메라 및 LiDAR 융합과 실차 통합 순서는 [방향 A](camera-lidar-navigation-plan.md)를 따른다.

ROS1/ViNT 코드와 training은 **이전 팀 저장소**(`kimdonggue/-Hansla-MRider-Autonomous-Driving`)에 보존하고
이 fork로 옮기지 않았다. 따라서 이 fork에는 `ros1_ws/`가 없고 `COLCON_IGNORE`도 필요하지 않다.
기존 자료를 삭제하거나 학습량을 변경하지 않는다. 학습 알고리즘을 제외한 현재 결정 때문에 ViNT 노드는 활성 ROS2로 이식하지 않았다.
기존 ROS1 실험 결과는 역사적 증거이며 ROS2 결과는 [새 검증 기록](ros2-validation.md)만 사용한다.

주의: 기존 경로 갱신 정책과 모의 입력 자동 복구는 유지한다. 최종 실차 게이트는 fault latch·재허가,
LiDAR 관측 유효성·충돌 거리·DBW 상태를 확인해야 한다. 단순 ROS2 전환으로 이 조건이 충족되지는 않는다.

후속 진행(2026-11-17): Nav2·모의 scan/지도·안전 게이트가 추가되었다. [센서 시뮬레이션](navigation-simulation-guide.md)과 [검증 기록](navigation-simulation-validation.md)을 참조한다. 실차/SLAM 미검증 경계는 유지한다.
