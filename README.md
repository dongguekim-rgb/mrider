# MRider — 팀 W·S 자율주행

**ROS2 Humble에서 카메라와 RPLiDAR A2M4-R2만으로 위치·지도를 추정하는 규칙 기반 Navigation**을 구현·검증 중입니다.
학습 모델과 Ground Truth 지도·위치는 주행 입력으로 사용하지 않습니다. 모의 차량과 센서는 실제 차량 검증을 대체하지 않습니다.

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

`indoor_demo`·`turns_demo` 등 초기 데모는 정답 지도·위치를 사용하는 계획/제어 기준 시험입니다.
센서 기반 주행은 `sensor_only_demo`·`sensor_turns_demo`·`large_office_demo`로 실행합니다.
각 설명서의 검증 날짜·결과·미검증 항목을 확인하세요. 실제 카메라/A2와 Teensy DBW 통합 주행은 아직 남아 있습니다.

교수님이 제공한 수업·과제·채점·설계·구매 문서는 원문을 보존합니다. 문서 정리는 팀에서 작성한 중복·임시 자료에만 적용합니다.

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
