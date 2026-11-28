# MRider — 팀 W·S 자율주행

**ROS2 Humble의 규칙 기반 Navigation**입니다. LiDAR scan으로 위치와 지도를 추정하고,
카메라 영상·보정값·LiDAR 기하로 바닥/벽/장애물 후보와 통행 가능 여부를 검사합니다.
카메라는 visual odometry를 계산하지 않습니다. 실물 LiDAR는 **RPLiDAR A2M4-R2**이며,
현재 검증에는 합성 RGB·LaserScan과 실제 `slam_toolbox`·Nav2를 연결한 **2D 센서 시뮬레이션**을 사용합니다.
학습 모델과 Ground Truth 지도·위치는 주행 입력으로 사용하지 않습니다. GT는 평가에만 사용합니다.

**검증 기준일: 2026-11-27.** 주행 소스 `2134ab7`의 선택 회귀 **282/282 PASS**입니다.
같은 소스의 두 GUI 실행은 별도 결과이며 테스트 수에 합산하지 않습니다.

| 실행 | 직진 → 90도 좌회전 → 90도 우회전 목표 오차 | 장애물 정지·자동 재개 | 충돌 |
|---|---|---|---|
| GUI 재검증 | 0.018992 / 0.155734 / 0.119914 m | 4회 | 없음 |
| 추가 녹화 실행 | 0.096136 / 0.226672 / 0.079524 m | 2회 | 없음 |

[RViz 녹화 MP4 — 4분 10.5초](docs/build/evidence/sensor-navigation-video-2026-11-27/run-01/rviz-navigation.mp4) ·
[실제 측정값·원본 XML·시행착오](docs/experiment-log.md#current-validation).
차량 1의 축거 **0.533 m**, 윤거 **0.435 m**, 바퀴 반경 **0.101 m**, 최소 회전반경 **2.10 m**를 적용했습니다.
물리 조향 한계는 약 **0.248561 rad**이며, 부하 조건의 반복 기동·장시간 정차 관측,
Gazebo 물리 twin·실센서·Teensy 통합은 남은 검증입니다. 저장소 전체 CI나 실차 주행의 합격을 뜻하지 않습니다.

## 시뮬레이션 실행

팀 작업 폴더는 `mrider-team`, 브랜치는 `sw/software-autonomy`입니다.
새 PC는 [설치·빌드 안내](docs/new-pc-setup.md), 기존 컨테이너의 코드 전달은
[Docker 동기화 안내](docs/existing-humble-container.md)를 먼저 따르세요.
호스트와 컨테이너의 팀 폴더는 별도 checkout이며 저장만으로 동기화되지 않습니다.

아래는 Humble 의존성과 팀 코드가 준비된 **기존 `ros2_humble`**에서 큰 맵을 실행하는 절차입니다.
호스트 터미널에서:

```bash
docker exec -it ros2_humble bash
```

컨테이너의 첫 터미널에서 빌드 후 실행합니다. 같은 domain의 이전 주행 launch는 먼저 종료합니다.

```bash
cd /root/mrider-team
source ros2_ws/setup_env.sh
export ROS_DOMAIN_ID=192
cd ros2_ws
colcon --log-base log-docker build --build-base build-docker --install-base install-docker \
  --symlink-install --packages-up-to mrider_navigation_sim --cmake-args -DBUILD_TESTING=ON
source install-docker/setup.bash
ros2 launch mrider_navigation_sim large_office_demo.launch.py \
  vehicle:=1 auto_resume:=true actor_time_offset_s:=227.0 rviz:=true
```

GUI에는 호스트 DISPLAY/X 인증과 컨테이너 연결이 필요합니다.
같은 domain의 RViz가 이미 열려 있거나 화면이 없는 환경이면 `rviz:=false`로 실행합니다.
이 launch는 2D 센서 시험이며 Gazebo 창을 띄우지 않습니다. 공식 Gazebo 절차는 [원본 twin 문서](docs/run-the-twin.md)에 있습니다.

두 번째 터미널도 `docker exec -it ros2_humble bash`로 들어간 뒤:

```bash
cd /root/mrider-team
source ros2_ws/setup_env.sh
source ros2_ws/install-docker/setup.bash
export ROS_DOMAIN_ID=192
ros2 service call /mrider_sim/arm std_srvs/srv/Trigger '{}'
# success: true 확인 후
ros2 service call /mrider_sim/start std_srvs/srv/Trigger '{}'
ros2 topic echo /mrider_sim/mission_state
```

Nav2 초기화가 완료돼야 출발합니다. 기본 목표는 `(24, 0)` m이며 중간 목표를 거쳐 전진합니다.
**영상처럼 세 목표·좌우회전까지 검사하려면** 수동 arm/start 대신 [세 목표 검사 절차](docs/large-office-navigation.md#three-goal-check)를 사용하세요.
장애물 정지만 조건부 자동 재개하고, 센서 고장·운영자 disarm은 자동으로 해제하지 않습니다.
중단은 `ros2 service call /mrider_sim/disarm std_srvs/srv/Trigger '{}'`로 요청하고 launch 터미널에서 Ctrl+C로 종료합니다.

## 자동 검증

수동 주행 launch를 종료하고, 위 환경을 source한 컨테이너의 `/root/mrider-team/ros2_ws`에서 실행합니다.

```bash
export MRIDER_TEST_VEHICLE=1
export MRIDER_SIM_REPORT_DIR="$(mktemp -d /tmp/mrider-sensor-tests.XXXXXX)"
colcon --log-base log-docker test --build-base build-docker --install-base install-docker \
  --packages-select mrider_navigation_sim --event-handlers console_direct+ \
  --ctest-args -R 'test_sensor_only_launch|test_sensor_turns_launch|test_large_office(_turns|_dynamic)?_launch' -j1 --output-on-failure
colcon test-result --test-result-base build-docker/mrider_navigation_sim/test_results --verbose
```

각 시험의 목표 도착·충돌 없음·조향 제한·정지/재개·고장 latch와 프로세스 종료를 확인합니다.
과거 XML과 이번 결과를 구분하고, CTest wrapper를 개별 시험 수에 더하지 않습니다.
282개 회귀를 실행한 전체 선택 명령과 기록은 [실험 기록](docs/experiment-log.md)에 있습니다.

## 문서 안내

| 필요한 정보 | 문서 |
|---|---|
| 교수님 원본 프로젝트·수업 안내 | [프로젝트 소개](docs/index.md), [수업 자료](docs/course/index.md) |
| 설치·colcon 빌드·컨테이너 | [새 PC](docs/new-pc-setup.md), [기존 Docker](docs/existing-humble-container.md) |
| 알고리즘 입출력·파일 책임 | [현재 아키텍처](docs/architecture.md), [폴더 구조](docs/folder-tree.md) |
| 카메라 인지·SLAM·센서 기반 주행 | [인지](docs/classical-perception.md), [센서 Navigation](docs/sensor-only-navigation.md) |
| 좌우 우회·횡단 정지·재개 | [복합 시나리오](docs/sensor-turns-simulation.md) |
| 60×40 m 실내 맵·장거리 검증 | [맵 설계](docs/large-office-simulation.md), [단계별 주행](docs/large-office-navigation.md) |
| 현재 성공/실패·시행착오 이력 | [실험 기록](docs/experiment-log.md) — 중요한 실패·수정·재검증 때마다 갱신 |
| 실차 준비·검증 | [실환경 설명서](docs/real-world-test-guide.md), [차량 실측값](docs/build/vehicle1-measurements-2026-11-23.md) |
| 팀 역할·인터페이스 규칙 | [.github/README.md](.github/README.md) |

`indoor_demo`·`turns_demo` 등 초기 데모는 정답 지도·위치를 사용하는 [계획/제어 기준 시험](docs/simulation-test-guide.md)입니다.
센서 기반 주행은 `sensor_only_demo`·`sensor_turns_demo`·`large_office_demo`로 실행합니다.
각 설명서의 검증 날짜·결과·미검증 항목을 확인하세요. 실제 카메라/A2와 Teensy DBW 통합 주행은 아직 남아 있습니다.

교수님이 제공한 수업·과제·채점·설계·구매 문서와 팀 규칙은 보존합니다.
팀 문서는 실행·구조·실험 절차와 시행착오 기록을 중심으로 두고, 중복된 과거 결과 설명은 실험 기록에 통합합니다.

## 공식 플랫폼 배경

아래 내용은 upstream 플랫폼 설계의 배경입니다. 공식 wheel odometry·IMU·학습 경로를 팀 센서 실험의 실행 입력으로 사용한다는 뜻은 아닙니다.

<p align="center">
  <img src="docs/images/mrider-small.png" alt="MRider">
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <img src="docs/images/mitt-small.png" alt="MITT">
</p>

<p align="center">
  📖 <strong>Documentation site: <a href="https://jrkwon.github.io/mrider/">jrkwon.github.io/mrider</a></strong>
</p>

**MRider** converts a kids ride-on, remote-controllable electric vehicle into a self-driving-ready car with a full **Drive-By-Wire (DBW)** system. The vehicle's nickname is **MITT** (Michigan Intelligent Transportation Tech), inspired by KITT from *Knight Rider*.

MRider is a **research and education platform**: rigorous enough for autonomous-driving research (mapping, navigation, end-to-end learning), documented well enough — [RoboRacer](https://roboracer.ai/)-style — for courses and external replication. It is the fourth generation of a platform line: [AVCS Kit](https://ieeexplore.ieee.org/document/8095696) (2017) → [Ridon Vehicle](https://www.mdpi.com/1996-1073/14/23/8039) (2021) → [OSCAR](https://github.com/bimilab/project-oscar/tree/devel_mrover) (2022) → [B-MROVER](https://github.com/jrkwon/mrover), whose validated ROS 2 stack MRider reuses.

> **Status:** design complete and consensus-reviewed. The **digital twin runs end to end** —
> Gazebo Harmonic, `slam_toolbox` mapping, Nav2 reaching goals ([run the twin](docs/run-the-twin.md)).
> Hardware sourcing is underway ([order log](docs/order-log.md)). Taught as
> [자율주행미들웨어응용](docs/course/syllabus.md), Fall 2026.

## System at a Glance

| Subsystem | Design |
|---|---|
| Chassis | **12 V single-seater** ride-on (Land Rover Defender class, 98 × 56 × 47 cm, ~10 kg), parent-remote, reversible conversion — [ADR D-R](docs/design/vehicle.md) |
| DBW | **Single Teensy 4.1 running micro-ROS** + Cytron MDDS30 in independent R/C mode — [ADR D3](docs/design/adr-dbw-architecture-review.md) |
| Steering | **Load-side absolute angle sensor** on the kingpin axis, ≥200 Hz position loop **on the Teensy**; ±22.5° travel, R_min 1.52 m — [ADR B / ADR E](docs/design/dbw.md) |
| Odometry | One quadrature encoder on one drive-motor shaft — **PPR must be measured, never inherited** (finding F7) — roll-out calibrated, fused with the IMU in an EKF — [ADR C](docs/design/dbw.md) |
| Sensors | Front camera + 2D LiDAR (minimum); BNO085-class IMU; GNSS/RTK as a growth path. The LiDAR scans 360° from the top of the mast; nothing sees below its scan plane |
| Compute | Onboard laptop (own battery) running ROS 2 Humble — SLAM, Nav2, and an end-to-end NN pipeline |
| Safety | Hardware E-stop, relay MUX (default = stock), hardware RC signal MUX, SBUS override — **three of the four work with the firmware dead** — plus failsafe matrix + FMEA |

Estimated cost: **~$725 Tier 1 / ~$910 both tiers**, contingency included (see [BOM](docs/design/bom.md); line items sum to $658 / $826).

## Documentation

### Design

| Document | Contents |
|---|---|
| [docs/design/overview.md](docs/design/overview.md) | Project goals, requirements, and resolved design decisions |
| [docs/design/architecture.md](docs/design/architecture.md) | System architecture, command/feedback flows, power tree, timing contract |
| [docs/design/vehicle.md](docs/design/vehicle.md) | Chassis selection criteria, candidate models, ADR D and its reversal to the 12 V single-seater |
| [docs/design/dbw.md](docs/design/dbw.md) | The core DBW design: steering smart-servo, throttle, encoders, interface contract |
| [docs/design/safety.md](docs/design/safety.md) | Failsafe matrix, authority arbitration, E-stop semantics, FMEA, bring-up protocol |
| [docs/design/sensors.md](docs/design/sensors.md) | Camera and LiDAR ADRs, IMU, GNSS/RTK path, mounting, laptop criteria |
| [docs/design/calibration.md](docs/design/calibration.md) | Steering, odometry, camera/LiDAR extrinsics, IMU, time-sync procedures |
| [docs/design/software.md](docs/design/software.md) | ROS 2 Humble stack, mrover reuse map, topic/TF contract, SLAM/Nav2/NN plan |
| [docs/design/bom.md](docs/design/bom.md) | Itemized bill of materials with minimum- and full-tier totals |

### Build & Learn (RoboRacer-style)

- **[Build guide](docs/build/index.md)** — BOM → mechanical → electrical → firmware → software → bench test → manual drive → autonomous.
- **[Learn curriculum](docs/learn/index.md)** — ROS 2 intro → DBW → safety → perception → SLAM → Nav2 → behavior cloning → capstone.

## Lineage & Related Work

**Previous work (documents):**
[Scaled-vehicle platform thesis (UM Deep Blue)](https://deepblue.lib.umich.edu/bitstreams/30717b2a-7c57-4604-95d6-ea3918857a89/download) ·
[Ridon Vehicle, *Energies* 2021](https://www.mdpi.com/1996-1073/14/23/8039) ·
[AVCS Kit, IEEE AFRICON 2017](https://ieeexplore.ieee.org/document/8095696)

**Previous work (code):**
[jrkwon/mrover](https://github.com/jrkwon/mrover) ·
[bimilab/project-oscar (devel_mrover)](https://github.com/bimilab/project-oscar/tree/devel_mrover)

**Similar work:**
[SKKUAutoLab/autolab_kingocar](https://github.com/SKKUAutoLab/autolab_kingocar) ·
[SKKUAutoLab/H-Mobility-Autonomous-Advanced-Course](https://github.com/SKKUAutoLab/H-Mobility-Autonomous-Advanced-Course) ·
[RoboRacer (F1TENTH)](https://roboracer.ai/)

---

Bio-Inspired Machine Intelligence Lab (BIMI), University of Michigan-Dearborn
