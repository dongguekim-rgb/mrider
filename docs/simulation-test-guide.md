# ROS2 Pure Pursuit 기준 시험

건물 내부 지도와 움직이는 장애물은 [실내 동적 시뮬레이션 설명서](indoor-simulation-guide.md)를 참고하세요.

이 문서는 알려진 경로와 모의 정답 위치를 받는 기본 Pure Pursuit 데모의 검사 절차다.
현재 카메라·LiDAR 위치 추정 주행은 [센서 시험](sensor-only-navigation.md),
알려진 지도·정답 위치 조건의 Nav2 장애물/고장 시험은 [Navigation 기준 시험](navigation-simulation-guide.md)을 따른다.

선행: [새 PC 설치·colcon 빌드](new-pc-setup.md). 실제 장치·firmware·학습 모델은 필요하지 않다.
가상 축거 0.65m, 속도 상한 0.8m/s, 조향 상한 0.3927rad이며 실차 보정값이 아니다.

## 1. 독립 제어·폐루프 실행

```bash
source /opt/ros/humble/setup.bash
source ~/mrider-team/ros2_ws/install/setup.bash
ros2 launch mrider_autonomy demo.launch.py path_kind:=straight
```

종료 후 `path_kind:=curve`로 반복한다. 목표에 가까워지면 감속하고 허용 거리 0.12m 내에서 정지해야 한다.
모의 시작점은 (0,-0.2), 직선 목표는 (8,0), 곡선 목표는 (4,4)다.
알고리즘만 사용할 때는 `autonomy.launch.py`를 실행하고 Path/Odometry를 별도로 공급한다.
입력 publisher가 끊기면 0.5s 수신/원본 timestamp 감시로 정지한다.

```bash
ros2 topic echo /mrider_sim/command
ros2 topic echo /mrider_sim/control_status
ros2 topic hz /mrider_sim/odometry
ros2 topic info /mrider_sim/path --verbose
```

입력 subscriber는 best-effort/volatile/depth 1이어서 센서 publisher와 연결 가능하다.
명령 출력은 reliable/volatile/depth 1이다. 경로/odom은 계속 새로운 timestamp로 갱신해야 한다.
미래·반복·역행 timestamp, 잘못된 frame, 비평면 pose, 비정규 quaternion, NaN/Inf는 정지 조건이다.
후륜축 frame은 ROS2에서 `base_link`다. `rear_axle` 문자열만 바꾸지 말고 실제 원점·축을 확인한다.

## 2. Gazebo Classic + RViz2

```bash
ros2 launch mrider_autonomy visual_demo.launch.py
```

Gazebo는 모의 차량의 위치를 표시하며, RViz2는 목표 경로·차체·누적 궤적을 표시한다.
Gazebo가 `/clock`을 발행하고 노드들이 이를 사용한다. Pause 시 steady-clock 감시는 정지 명령 발행을 계속한다.
RViz Fixed Frame은 `map`, 경로는 `/mrider_sim/path`, MarkerArray는 `/mrider_sim/markers`다.

```bash
# RViz2만 실행, system time 사용
ros2 launch mrider_autonomy visual_demo.launch.py gazebo:=false
# 디스플레이 없는 서버: Gazebo server만, GUI와 RViz2 제외
ros2 launch mrider_autonomy visual_demo.launch.py gui:=false rviz:=false
```

**기구학 시각화다.** 충돌·마찰·제동·카메라 영상·LiDAR scan을 시뮬레이션하지 않는다.
Gazebo 경로 표시는 RViz2의 Path로 제공하며 Gazebo 내부에는 차량 모형만 표시한다.
GUI 문제는 그래픽 환경과 함께 기록하며 headless 시험과 구분한다.
시각화 실행 중 동일 ROS_DOMAIN_ID·setup 환경의 다른 터미널에서 다음 명령으로 차량 pose·TF·마커를 검사한다.

```bash
python3 ~/mrider-team/tools/ros2/check_visualization.py
```

## 3. 자동 검증

```bash
cd ~/mrider-team/ros2_ws
colcon test --executor sequential --ctest-args -j1 --output-on-failure
colcon test-result --verbose
ros2 run mrider_autonomy offline_demo
```

실토픽 launch_testing은 조향 부호/제한, 잘못된 frame·NaN·timestamp,
입력 중단·복구, 시계 정지·역행·복구, 직선/곡선 폐루프 감속·목표 정지를 검사한다.
시험은 공통 토픽 이름을 사용하므로 순차 실행하고 다른 데모를 같은 ROS_DOMAIN_ID에서 실행하지 않는다.
이전 결과·오차와 증거 범위: [초기 기준 시험 기록](experiment-log.md#legacy-evidence).

## 4. 실차로 전환하기 전

`reference_preview`는 단위 변환 관찰용이며 `/mitt/dbw/command`도 실차 controller reference도 아니다.
임의 remap으로 구동하지 않는다. [Teensy 계약](teensy-ros2-integration.md)의 단일 명령 경로와
[실차 단계별 검증](real-world-test-guide.md)을 먼저 완료한다.
