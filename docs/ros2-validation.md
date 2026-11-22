# ROS2 검증 기록 — 2026-11-17 (KST)

이 문서는 ROS2 이식 당시 기본 제어 시험 기록이다. 후속 센서·Nav2 확장과 전체 재검증은 [Navigation 검증 기록](navigation-simulation-validation.md)을 따른다.

이번 결과는 ROS2 이식 코드의 실제 실행 결과다. 이전 ROS1 결과와 합산하지 않는다.
그 ROS1 기록은 **이전 팀 저장소**(`kimdonggue/-Hansla-MRider-Autonomous-Driving`)의 `docs/validation.md`에 있고 이 fork로 옮기지 않았다.
호스트는 Linux Mint 20.2 / Python 3.8 / Noetic·Foxy이며 Humble은 Docker의 Ubuntu 22.04 / Python 3.10에서 실행했다.
호스트 ROS 설치와 개인 Lab 저장소는 변경하지 않았다.

## 환경과 명령

- 이미지: `osrf/ros:humble-desktop-full-jammy`
- 기본 이미지 digest: `sha256:592f97da6b871a438fdb97f86b422ab9f80657e80efa7b98d258894738c8a0c0`
- rclpy `3.3.21-1jammy.20260724.022150`, gazebo_ros `3.9.0-1jammy.20261110.093533`, ackermann_msgs `2.0.2-3jammy.20261105.200946`
- 테스트 ROS_DOMAIN_ID=187, 별도 시각화 ROS_DOMAIN_ID=188. 실제 장치는 컨테이너에 연결하지 않음.

실행 명령은 [verify.sh](../tools/ros2/verify.sh)의 colcon build/test/test-result와 offline_demo다.
가상 치수, 50Hz, frame `map`/`base_link`로 검사했다. build와 test는 모두 성공했다.

## 자동 테스트

| 범위 | 개별 테스트 수 | 결과 |
|---|---:|---|
| ROS 없는 Pure Pursuit core | 19 | PASS |
| ROS2 메시지 경계·단위 변환 | 13 | PASS |
| 실제 토픽 계약 | 6 | PASS |
| 실제 토픽 곡선 폐루프 | 1 | PASS |
| 실제 토픽 직선 폐루프 | 1 | PASS |
| /clock 정지·역행·복구 | 1 | PASS |
| launch 노드 정상 종료 | 4 | PASS |
| **개별 합계** | **45** | **실패·오류·skip 0** |

CTest 그룹 6개 모두 PASS. `colcon test-result`는 wrapper 6개를 포함해 **51 tests, 0 errors, 0 failures, 0 skipped**로 보고한다.
호스트 ROS 없는 unittest 19개도 별도 통과했으며 위 합계에 중복 가산하지 않는다.
핵심 검사: 조향 부호/상한·감속·목표 정지·NaN/Inf·잘못된 frame/quaternion·미래/오래된/반복 timestamp·입력 중단/복구.
ROS clock이 멈춰도 wall-time timer가 정지를 발행하고, 역행 후 새 입력으로 복구함을 실토픽으로 검사했다.

결과 XML은 `docs/results/ros2/`에 보관한다. 목표 오차와 전체 실행 로그도 같은 디렉터리에 기록한다.

## 목표 도달 오차

| 실행 | 직선 | 곡선 |
|---|---:|---:|
| ROS 없는 결정론적 폐루프 | 0.1195742744m | 0.1188218188m |
| 최종 Humble verify.sh 실토픽 실행 | 0.1188984451m | 0.1181357139m |

허용 오차는 0.12m이며 최종 속도는 0m/s다. ROS 없는 최대 절대 조향각은 0.3927rad다.
실토픽 수치는 스케줄링에 따라 달라진다. 표는 종료 경쟁 수정 후 최종 실행 값이며 이전 실행은 보관 로그와 구분한다.
위 결과는 실차 오차·정지거리·장애물 회피 성능이 아니다.

## 시각화·launch 확인

- `visual_demo.launch.py gui:=false rviz:=false`: Gazebo server와 제어·모의 차량 실행.
- [check_visualization.py](../tools/ros2/check_visualization.py): GetEntityState 차량 존재, Odometry/Marker/TF 수신, pose 차이 **0.0m**, Marker 2개, `map → base_link` PASS.
- RViz2: Xvfb에서 설정 파일을 열어 12초 실행, OpenGL 4.5 초기화 확인. 시험 종료 SIGINT 후 가상 X11 연결 종료 로그가 남음.
- 알고리즘/demo/시각화/센서/agent launch의 `--show-args` 성공. 센서/agent는 장치 없이 인자 구성만 검사함.

실행 중 컨테이너 사용자에게 `/.ros`, `/.gazebo` 쓰기 권한이 없어 최초 launch가 실패했다.
테스트는 ROS_LOG_DIR를 쓰기 가능한 `/tmp`로 지정해 해결했고, Gazebo/RViz 시각화는 컨테이너 기본 사용자로 실행했다.
검증 스크립트의 `set -u`는 ROS 환경 source 후 활성화하도록 수정하여 unbound 변수 오류를 해결했다.
Gazebo의 오디오 장치 없음 경고는 남았으나 pose 서비스 검사는 통과했다.
추가로 SIGINT 처리 중 DDS context가 먼저 해제되어 subscriber take가 실패하는 종료 경쟁을 발견했다.
기본 rclpy signal shutdown 대신 Python 종료 신호로 spin을 끝낸 뒤 context를 정리하도록 수정했다.
launch_testing의 post_shutdown 검사 4개를 추가했으며 최종 실행에서 모든 노드 exit code 0을 확인했다.
`pre-fix-shutdown.txt`와 `initial-colcon-test.txt`는 수정 전 기록이며 최신 합격 XML과 구분한다.

## 남은 검증

- 팀원 실제 데스크톱에서 Gazebo GUI·RViz2 화면과 그래픽 드라이버 확인.
- 실제 A2M4-R2의 baud·scan 모드·거리·시각·가림 및 카메라 보정.
- Teensy firmware·mitt_msgs·micro-ROS agent 버전/전송, 50Hz status, 실제 조향/속도 feedback.
- SLAM/EKF/Nav2·충돌/권한/재허가 게이트·실측 차량 설정·실차 launch 구현과 검증.
- M2/M3 물리적 정지·RC·E-stop·USB 상실·MCU reset/hang·저속 현장 시험.

위 항목은 NOT RUN 또는 미구현이며 ROS2 빌드 성공으로 대체하지 않는다.

## 정적 검토

Markdown 24개, 로컬 링크 151개, Bash 예제 41개 검사 PASS.
ROS2 Python/launch AST, package.xml 구문, verify.sh 구문 및 git diff --check PASS.
현행 ros2_ws 소스에는 rospy·roslaunch·Catkin·ROS1 Time.to_sec 의존성이 없다.
