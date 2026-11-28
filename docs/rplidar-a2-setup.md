# RPLiDAR A2M4-R2 — 방향 A 센서 준비·검증

확인일: 2026-11-16. **팀 사용 모델은 RPLiDAR A2M4-R2**(사용자가 본체 라벨 확인)다.
firmware·adapter·serial 및 실제 연결 호환성은 별도 확인한다.
아래는 ROS2 Humble용 준비 절차이며 이번에 장치 연결·빌드·scan 수신을 실행한 것은 아니다.
카메라 모델, 장치 serial/adapter, 실측 장착 위치·자세는 별도 확정해야 한다.
학습 없는 방향 A의 구조는 유지하며 센서 모델에 따른 통신·탐지 범위를 다시 검증한다.

## 공식 ADR과 팀 선택

[MRider sensors §2.2](https://jrkwon.github.io/mrider/design/sensors/#22-adr-lidar-recommendation-for-min-and-full-tiers)의
최소 tier G4/full tier S3 권장과 팀의 A2 선택을 구분한다.
이전 S2의 1000000 baud·DenseBoost·30m·IP65 정보를 A2에 적용하지 않는다.
탐지 거리·주기·환경 조건은 세부 모델의 공식 사양과 실측으로 확정하며, 이를 확인하기 전 실차 운용 범위를 정하지 않는다.

## A2M4-R2 설정 근거와 남은 확인

[제조사 A2M4 데이터시트](https://bucket-download.slamtec.com/004eb70efdaba0d30a559d7efc60b4bc6bc257fc/LD204_SLAMTEC_rplidar_datasheet_A2M4_v1.0_en.pdf)의
검색 색인에 표시된 UART 사양은 **115200 baud**다. 이번 조회에서 PDF 본문 다운로드는 HTTP 403으로 실패했다.
R2 별도 변경 사양은 확인하지 못했으므로 이 값은 A2M4 자료 기준이며, 연결 성공/장치 정보/scan 수신으로 실기 확인한다.

[고정한 공식 드라이버 버전](https://github.com/Slamtec/sllidar_ros2/tree/34300099fadfc772965962dec837bf436706188f/launch)에는
A2M4 전용 launch가 없다. 다른 모델의 launch를 A2M4 전용으로 간주하지 않고 아래처럼 공식 노드를 직접 실행한다.
[노드 소스](https://github.com/Slamtec/sllidar_ros2/blob/34300099fadfc772965962dec837bf436706188f/src/sllidar_node.cpp)에서
`serial_baudrate`, `frame_id`, `inverted`, `angle_compensate` 파라미터와 빈 `scan_mode`의 typical scan 선택 분기를 확인했다.
`scan_mode`를 지정하지 않아 장치/SDK의 typical 모드를 선택하도록 한다. 특정 모드 이름을 강제하지 않는다.
이것은 소스에 근거한 **실기 검증용 예제**이며 A2M4-R2와 이 SDK 조합의 호환성 합격을 의미하지 않는다.
드라이버가 출력하는 scan frequency 설정값도 실측 주기가 아니므로 `/scan` 수신 주기를 별도 측정한다.

## 새 PC 설치·빌드

Ubuntu 22.04/ROS2 Humble이 준비된 별도 터미널을 사용한다. ROS1 workspace와 분리한다.
아래 workspace는 센서 검사 전용이며 차량 제어·micro-ROS agent를 시작하지 않는다.
기존 같은 이름의 checkout이 있으면 덮어쓰지 말고 로컬 변경·버전을 먼저 확인한다.

```bash
source /opt/ros/humble/setup.bash
sudo apt-get install git python3-colcon-common-extensions python3-rosdep ros-humble-rviz2
export A2_WS="$HOME/mrider-a2-ws"
mkdir -p "$A2_WS/src"
git clone https://github.com/Slamtec/sllidar_ros2.git "$A2_WS/src/sllidar_ros2"
git -C "$A2_WS/src/sllidar_ros2" checkout 34300099fadfc772965962dec837bf436706188f
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
cd "$A2_WS"
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
colcon test --packages-select sllidar_ros2
colcon test-result --verbose
source install/setup.bash
ros2 pkg prefix sllidar_ros2
```

통과: build 성공, 의존성 충족, package 경로 확인. 테스트가 없으면 0개를 장치 합격으로 해석하지 않는다.
이 커밋은 조회한 소스 버전이며 이 팀의 Humble 실기 검증 버전은 아직 아니다.

## 센서 단독 실행

장치 모델·adapter·전원을 확인하고 `/dev/serial/by-id/`에서 A2의 장치를 식별한다.
Teensy 장치와 혼동하지 않는다. serial 장치 읽기/쓰기 권한과 안정적인 경로를 준비한다.
필요 시 장치별 udev/dialout 설정을 적용하고 재로그인 후 확인한다. 포트 이름을 추측하여 고정하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/mrider-a2-ws/install/setup.bash"
ls -l /dev/serial/by-id/
# 사용자 확인 본체 모델: A2M4-R2; A2M4 자료 기준 115200 baud
read -r -p 'A2M4-R2 serial device path: ' A2_PORT
test -c "$A2_PORT" && test -r "$A2_PORT" && test -w "$A2_PORT" || exit 1
ros2 run sllidar_ros2 sllidar_node --ros-args \
  -p channel_type:=serial -p serial_port:="$A2_PORT" \
  -p serial_baudrate:=115200 -p frame_id:=laser \
  -p inverted:=false -p angle_compensate:=true
```

다른 동일 환경 터미널에서 다음 명령을 각각 실행하고 연속 관찰은 Ctrl+C로 종료한다.

```bash
ros2 topic info /scan --verbose
ros2 topic hz /scan
ros2 topic echo /scan --once
rviz2
```

단독 관찰 시 RViz Fixed Frame은 `laser`, LaserScan은 `/scan`이다.
차량 통합 시 실측 URDF의 `base_link → laser` TF를 사용한다. `lidar_link`를 쓰면 driver frame과 URDF를 함께 맞춘다.
임의의 영점 static TF를 실측 calibration으로 간주하지 않는다. driver 중복 실행·중복 TF도 금지한다.

## 실기 합격 기록

- 모델·serial·firmware·adapter·드라이버 커밋·선택 scan mode·baud와 시작 로그 보존.
- 실제 `angle_increment`, `scan_time`, `time_increment`, `range_min/max`, 유효 점 수·누락률·최대 지연 기록.
- 알려진 벽 거리·전방/좌우 표적을 RViz와 대조하고 scan frame·방향·단위를 확인.
- 낮은 물체·의자 다리·검은 무광·유리·실내/실외 햇빛 조건의 탐지 누락을 기록.
- 차체·mast·배선에 가려지는 sector를 실측. 공식 사이트의 후속 장착 수정(상단 LiDAR·전방 boom 카메라)을 참고하되 치수는 복사하지 않음.
- 구동 차단 상태에서 USB 단절·재연결 시험. scan 중단 시 정지 계층이 fault로 처리하고 운전자 재허가 없이 주행하지 않는지 후속 통합에서 확인.
- [M4 검증표](real-world-test-guide.md#sensor-calibration)의 CameraInfo·TF·overlay 및 bag 재생 검사 수행.

범위·주기·누락률·지연·거리 오차 합격값은 실제 장착과 시험 장소를 기준으로 사전 확정한다.
미정 기준이나 미측정 항목은 [실환경 양식](real-world-validation-template.md)에 NOT RUN으로 남긴다.
A2를 연결했다는 사실만으로 SLAM·장애물 정지·Nav2 통합 완료를 선언하지 않는다.

## 이번 문서 정정 검증

2026-11-16: Markdown 20개, 로컬 링크 150개, Bash 예제 57개의 구문 및 코드 블록 구분자 검사를 통과했다.
A2M4 데이터시트 검색 색인의 통신 설정 및 공식 고정 버전 노드 소스의 파라미터/scan 선택 분기를 대조했다.
PDF 본문은 403으로 확인하지 못했으며 R2 전용 사양·SDK 실기 호환성은 확인 과제로 남겼다.
ROS2 빌드·장치 연결·scan 수신·실차 시험은 NOT RUN이다.

## 팀 ROS2 launch 사용

센서 workspace와 팀 ros2_ws를 모두 빌드/source한 뒤 같은 노드를 팀 launch로 실행할 수 있다.
앞의 직접 노드 실행과 동시에 실행하지 않는다. 실기 미검증 범위는 동일하다.

```bash
source /opt/ros/humble/setup.bash
source ~/mrider-a2-ws/install/setup.bash
source ~/mrider-team/ros2_ws/install/setup.bash
read -r -p 'Verified A2M4-R2 serial device: ' A2_PORT
ros2 launch mrider_autonomy sensors.launch.py serial_port:="$A2_PORT"
```
