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
│   ├── mrider_navigation_sim/           # 센서 추정/분할/SLAM 연결/지역 목표/안전/세계
│   │   ├── sensor_core.py               # 강건 ICP·최근 두 기준 합의 복구
│   │   ├── moving_obstacles.py          # LiDAR 군집 추적·12초 예측 충돌 검사
│   │   ├── path_recovery.py             # 장애물 정지 뒤 남은 경로·차체 관측 재검증
│   │   ├── path_feasibility.py          # 지도 해상도·생성 경로·초기 추종 곡률 검사
│   │   ├── observed_map.py              # 반복된 최신 scan으로 점유 흔적 재검증
│   │   ├── synchronization.py           # 영상·scan 시각별 수신 버퍼
│   │   └── trace_navigation.py          # 주행에 관여하지 않는 평가용 진단 기록
│   ├── src/observed_obstacle_layer.cpp   # 반복 scan 근거로 잔류 장애물 정정
│   ├── observed_layer.xml               # Nav2 플러그인 등록
│   ├── src/managed_slam_node.cpp        # 설치된 SLAM 실행과 종료 전 worker 정리
│   ├── src/managed_controller_node.cpp  # stale costmap 대기 중 취소·정지·종료
│   ├── licenses/Apache-2.0.txt          # Nav2 파생 제어 코드 고지
│   ├── launch/                         # sensor_only, sensor_turns, large_office 및 기준 시험
│   ├── config/                         # Nav2/SLAM/카메라/차량/정지 규칙
│   ├── scripts/                        # 노드와 run_scenario
│   ├── rviz/                           # sensor_only.rviz(센서), navigation.rviz(기준 시험)
│   └── test/                           # 단위 + Nav2/고장/복구 실토픽
│       └── fixtures/                    # 실제 실패 시 보존한 합성 scan 재생 자료
├── ros2_ws/src/mitt_*/                 # 공식 패키지; vehicle1 측정 overlay 반영
├── tools/ros2/                        # 컨테이너와 검증 스크립트
└── docs/                              # 설치·시뮬레이션·센서·Teensy·검증
```

파일별 책임은 [architecture](architecture.md), 명령은 [새 PC 설명서](new-pc-setup.md),
검증 증거와 각 시험의 실행 안내는 [문서 안내](index.md)에서 찾는다.
팀 실험의 현재 성공/실패와 중요한 수정 이력은 [시행착오 기록](experiment-log.md)에 누적한다.
`build/install/log`와 `build-docker/install-docker/log-docker`는 Git 추적 대상이 아니다.

레거시 `ros1_ws/`(ROS1 Catkin)와 `training/`(ViNT·depth 학습)은 이 fork에 없다.
**이전 팀 저장소**(`kimdonggue/-Hansla-MRider-Autonomous-Driving`)에 보존되어 있으며
현행 ROS2 실행에 필요하지 않다.
