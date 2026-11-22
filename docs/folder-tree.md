# Folder Tree — 현행 ROS2

```text
mrider-team/
├── ros2_ws/src/mrider_autonomy/
│   ├── package.xml / CMakeLists.txt     # ament, 의존성, 설치/테스트
│   ├── mrider_autonomy/                # core, ROS2 boundary/nodes, simulator, visualization
│   ├── scripts/                       # ros2 run 진입점
│   ├── config/simulation.yaml          # 가상 치수·제어/timeout
│   ├── launch/                        # autonomy/demo/visual_demo/sensors/teensy_agent
│   ├── rviz/demo.rviz
│   ├── worlds/visual.world
│   └── test/                          # core/boundary + ROS2 launch tests
├── ros2_ws/src/mrider_navigation_sim/
│   ├── mrider_navigation_sim/           # geometry/indoor/world/safety/gate/mission/checks
│   ├── launch/                         # navigation_sim, indoor_demo, fault_test
│   ├── config/                         # Nav2와 가상 차량/정지 규칙
│   ├── scripts/                        # 노드와 run_scenario
│   ├── rviz/navigation.rviz
│   └── test/                           # 단위 + Nav2/고장/복구 실토픽
├── ros2_ws/src/mitt_*/                 # 공식 7개 패키지, upstream 원문 그대로
├── tools/ros2/                        # 컨테이너와 검증 스크립트
└── docs/                              # 설치·시뮬레이션·센서·Teensy·검증
```

파일별 책임은 [architecture](architecture.md), 명령은 [새 PC 설명서](new-pc-setup.md),
검증 증거는 [ROS2 validation](ros2-validation.md)을 따른다.
`build/install/log`와 `build-docker/install-docker/log-docker`는 Git 추적 대상이 아니다.

레거시 `ros1_ws/`(ROS1 Catkin)와 `training/`(ViNT·depth 학습)은 이 fork에 없다.
**이전 팀 저장소**(`kimdonggue/-Hansla-MRider-Autonomous-Driving`)에 보존되어 있으며
현행 ROS2 실행에 필요하지 않다.
