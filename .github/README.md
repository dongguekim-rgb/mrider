# MRider: 우리 팀 fork

> 이 저장소는 [`jrkwon/mrider`](https://github.com/jrkwon/mrider)의 **팀 fork**입니다.
> 원본 프로젝트 설명은 루트의 [README.md](../README.md)와
> [문서 사이트](https://jrkwon.github.io/mrider/)를 보세요.
> 이 페이지는 **우리 팀이 무엇을, 어디에, 언제까지 하는지**만 정리합니다.

| | |
|---|---|
| **우리 차** | vehicle `<n>`   ← 팀리드가 채움 · ROS domain = 10 + n |
| **팀리드** | 김동규 |
| **채점** | 이 fork의 **commit 기록**(산출물 + 개인별 기여) |
| **근거 문서** | [Teams & Tracks](../docs/course/teams.md) · [Grading](../docs/course/grading.md) · [Week 11](../docs/course/weeks/w11.md) · [Week 12](../docs/course/weeks/w12.md) |

---

## 0. 처음 한 번: 내 PC에 준비하기

```bash
git clone https://github.com/dongguekim-rgb/mrider.git mrider-team
cd mrider-team
git remote add upstream https://github.com/jrkwon/mrider.git

# commit이 "내 기여"로 잡히려면 GitHub 계정에 등록된 이메일이어야 합니다
git config --global user.name  "본인 이름"
git config --global user.email "GitHub에 등록된 이메일"

cd ros2_ws && colcon build --symlink-install
```

- `origin` = 이 팀 fork → **여기에 push합니다**
- `upstream` = 교수님 원본 → **받기만 합니다**
- collaborator 초대 메일을 **수락해야** push할 수 있습니다.

## 1. 매주, 그리고 merge demo 전마다: 최신으로 맞추기

```bash
git checkout main
git pull                         # 팀원 작업 받기
git fetch upstream
git merge upstream/main          # 교수님 변경 받기
cd ros2_ws && colcon build --symlink-install   # 반드시 다시 build
```

---

## 2. 규칙: 무엇을 어디에 두는가

| 무엇 | 어디에 | upstream(jrkwon)으로 PR? |
|---|---|---|
| **모든 코드**: `firmware/`, `ros2_ws/src/mitt_hardware/`, `ros2_ws/src/mitt_bench/`, test, sim world, charter | **이 fork** | ❌ 보내지 않음. 학기 말에 교수님이 한 팀 것을 골라 가져감(성적과 무관) |
| **우리 차 측정값**: `ros2_ws/src/mitt_description/config/vehicles/vehicle<n>.yaml` | upstream main | ✅ **이 파일 하나만** 바꾼 PR |
| **공통 interface** 변경 | upstream main | ✅ 바꿀 일이 생겼을 때만, 이유와 함께 |

### 🚫 fork에서 절대 바꾸지 않는 것 (세 팀 공통 interface)

- `mitt_msgs` 메시지 형식
- topic 이름, rate, frame (software.md §3.1, §3.3)
- Teensy pin 배치 (build step 4)
- ROS domain 규칙: vehicle N → domain **10 + N**
- `mitt_dimensions.yaml`의 **key 이름**
- `sim.launch.py`, `real.launch.py`, `nav2.launch.py`의 argument

이것들이 같아야 **어느 팀의 firmware든 어느 팀 차에서나** 돌아갑니다. 틀렸으면 고치지 말고 팀리드에게 알려 주세요. upstream에 PR로 냅니다.

### 🚫 고치지 않는 파일

- `mitt_dimensions.yaml`: 추정값으로 남겨 둡니다. 측정값은 `vehicle<n>.yaml`에 씁니다.
- `nav2_params.yaml`: footprint, inflation, turning radius는 `vehicle<n>.yaml`에서 자동으로 생성됩니다.
- 차마다 다른 숫자는 **코드에 하드코딩하지 말고** `vehicle<n>.yaml`에서 읽습니다.

### 작업 방식 (팀 규칙 제안)

```bash
git checkout -b <track>/<주제>       # 예: chassis/turning-radius, sw/mitt-hardware
# ... 작업, 각자 이름으로 commit ...
git push -u origin <track>/<주제>     # GitHub에서 이 fork의 main으로 PR
```
- 같이 작업했으면 commit 메시지 끝에 `Co-authored-by: 이름 <이메일>`을 붙입니다. 두 사람 모두 기록에 남습니다.
- 측정값, 로그, 그래프 같은 **근거는 commit합니다.** 기록 위치는 `docs/build/`입니다. "됐다"는 말은 근거가 아닙니다.

---

## 3. 일정

| 날짜 | 무엇 | 증명할 것 |
|---|---|---|
| **11/23 (월)** | **Merge 1**: Electrical + Chassis | 바퀴를 띄운 상태에서 firmware 제어로 지정한 조향각까지 꺾인다 |
| **11/30 (월)** | **중간발표(15%) + Merge 2**: Simulation + Software · Peer evaluation 1 | twin이 **측정 치수**로 돌고, functional test가 **CI에서 green** |
| **12/07 (월)** | **Merge 3**: (E+C) + Software | `mitt_hardware`가 실제 Teensy와 통신. **같은 launch가 sim과 실차에서 plugin만 바꿔서 돈다** (ADR-SW4) |
| **12/14 (월)** | **Merge 4**: 전체 | 실차 teleop → 복도 지도 작성 → Nav2 goal 주행 → acceptance gate 측정 |
| **12/21 (월)** | **Final demo** | |

> **merge 날짜는 미룰 수 없습니다.** 못 맞출 것 같으면 **미리** 팀리드에게 말해 주세요.
> 못 맞춘 항목은 **측정한 숫자 + 이유**로 보여 주면 됩니다.
> 이 과목의 원칙: *"Honest failure outscores a lucky success."*

---

## 4. Track별 할 일

### 🔧 C: Chassis & Measurement

**맡은 것:** 실제 차체, 조향 모터 마운트, 각도 센서, 구동 encoder, 장비 plate, 센서 mast. 그리고 무엇보다 **숫자**입니다.
**다른 모든 track이 이 track의 숫자를 기다리고 있습니다.**

| 우선순위 | 할 일 | 기한 |
|---|---|---|
| 1 | wheelbase, track width, **wheel radius(바닥에 하중을 실은 상태)** 측정. 값 + 측정 방법 + 신뢰도 | **11/23 Merge 1** |
| 2 | **최소 회전 반경**: 좌우 끝까지 꺾은 상태에서 각각 재고 **큰 값** 기록 ([calibration.md §1.5](../docs/design/calibration.md)). 이 값이 없으면 **실차 Nav2가 뜨지 않습니다** | 측정 즉시 |
| 3 | `vehicle<n>.yaml` 작성 → `python3 scripts/check_vehicle_dims.py` 통과 → **이 파일 하나만** upstream에 PR (아래 §5) | Merge 2 전 |
| 4 | Verification gate V1, V2, V3: 조향 모터 stall 전류, 센서 축 travel, 축 지름 | |
| 5 | [Build step 2](../docs/build/02-mechanical.md) 검증, calibration 상수 기록 | |

### ⚡ E: Electrical & Firmware (Hardware)

**맡은 것:** Teensy 4.1, micro-ROS, Sabertooth 모터 드라이버, relay MUX, RC signal MUX, 비상정지.

| 우선순위 | 할 일 | 기한 |
|---|---|---|
| 1 | **Merge 1 시연 항목** (아래 표) | **11/23** |
| 2 | `firmware/` (Teensy DBW 컨트롤러): 지금 **repo에 없습니다** | |
| 3 | `DbwCommand` / `DbwStatus`가 micro-ROS로 실제 오가게 하기 | Merge 3 (12/07) |
| 4 | Teensy에서 IMU 발행: `/imu/data` ≥ 100 Hz | |
| 5 | failsafe matrix를 바퀴 띄운 상태에서 시연 ([safety.md](../docs/design/safety.md)) | |
| 6 | Sim과 real 비교용 측정: 조향 범위, rate limit, 명령 지연 → Simulation에 전달 | Merge 2 전 |

**Merge 1 기준 (11/23):**

| 항목 | 기준 |
|---|---|
| 위치 추종 | 정상상태 오차 **≤ 1°**, 지속 진동 없음 |
| Loop rate | **≥ 200 Hz**, 실측 |
| 한계 동작 | ±22.5°를 넘게 명령하면 clamp |
| Stall 감지 | 바퀴를 잡으면 `DbwStatus.faults`에 stall bit |
| 전원 차단 | 조향이 잠기지 않고 풀림 |
| R/C 신호 끊김 | Teensy 펄스를 멈추면 모터 정지 |

> **E4 규칙:** ≤ 1°를 못 맞추면 튜닝을 계속하지 말고 **E4(전용 motion-controller daughterboard)로 전환**합니다.
> 미리 정해 둔 대안이라 감점이 없습니다. 감점 대상은 "거의 됐다"며 계속 튜닝하는 것입니다.

**최종 acceptance gate:** `/mitt/dbw/status` ≥ 50 Hz 유지 · 30분 동안 USB 끊김 0회 · override 두 층 모두 시연.

### 💻 W: Software & Autonomy

**맡은 것:** 실차에서 도는 ROS 2 stack.
**일정이 가장 빠듯한 track입니다.** `mitt_hardware`는 이번 주에 시작해야 합니다.

| 우선순위 | 할 일 | 기한 |
|---|---|---|
| 1 | `nav2.launch.py vehicle:=<n>`이 측정 회전 반경보다 5% 큰 반경으로 계획하는지 twin에서 확인 | Merge 2 (11/30) |
| 2 | Simulation과 함께 functional test와 CI 작업 | Merge 2 (11/30) |
| 3 | **`mitt_hardware`**: 실차용 `ros2_control` plugin. **가장 중요한 산출물**입니다. 지금 없어서 `real.launch.py`가 실행을 거부합니다 | **Merge 3 (12/07)** |
| 4 | **odometry node**: `DbwStatus`에서 bicycle model로 계산. 실차 EKF와 SLAM에 필요합니다 | Merge 4 (12/14) |
| 5 | RViz 설정, perception package(카메라 데이터 사용) | |

**Nav2와 SLAM 사용 시 주의:**
- sim/real과 Nav2에는 **같은 `vehicle:=<n>`**을 줍니다. 다르면 `WRONG VEHICLE` 경고가 나옵니다.
- 실차에서는 먼저 `MITT_VEHICLE=<n> source ros2_ws/setup_env.sh`를 실행합니다. 안 하면 Nav2가 뜨지 않습니다.
- `slam.launch.py`의 `use_sim_time` 기본값은 `true`입니다. 실차에서는 `use_sim_time:=false`를 줍니다.
- inflation radius가 0.90 m입니다. 좁은 틈은 거부될 수 있습니다(정상 동작).
- LiDAR는 360°라서 후진도 볼 수 있습니다. 다만 scan 면보다 낮은 물체는 못 봅니다. **실차 첫 후진은 조심해서** 진행합니다.
- 읽을 자료: [build step 8](../docs/build/08-autonomous.md) · [software.md §4.5–4.7](../docs/design/software.md)

### 🧪 S: Simulation & Validation

**맡은 것:** 시뮬레이션 world, sim의 정확도, 테스트 인프라, CI.
"다른 사람의 주장이 사실인지 판정하는 track"입니다.

| 우선순위 | 할 일 | 기한 |
|---|---|---|
| 1 | Chassis의 `vehicle<n>.yaml`로 `sim.launch.py vehicle:=<n>` 실행, **녹화** | Merge 2 (11/30) |
| 2 | **functional test** 작성: 지금 테스트 29개는 **전부 linter**입니다(코드 스타일 검사만 함) | Merge 2 |
| 3 | **CI**: GitHub Actions에서 ROS build와 `colcon test` 실행 | Merge 2 |
| 4 | **Sim과 real 비교**: 조향 범위, rate limit, wheelbase, 명령 지연이 **10% 이내**인지, 아니면 차이와 이유 | Merge 2 |
| 5 | 실제 test 복도와 같은 parametric world (지금은 기본 warehouse만 있음) | |
| 6 | `mitt_bench`: acceptance 측정 스크립트. **지금 없습니다** | |

**functional test 아이디어:**
- `vehicle:=<n>`으로 생성한 URDF가 정상이고 wheelbase가 측정값과 같은지
- `sim.launch.py`를 띄우면 controller 두 개가 active가 되는지
- 잘못된 `vehicle:=7`을 주면 **exit status 1**로 끝나는지
- 30° 조향 명령이 22.5°로 clamp되는지
- `mitt_controllers.yaml`과 URDF의 값이 일치하는지 (주석에는 "확인한다"고 써 있지만 실제로는 없는 검사)

---

## 5. 측정값 PR 보내는 법 (Chassis)

팀 코드가 섞이지 않도록 **upstream에서 출발한 branch**를 따로 만듭니다.

```bash
git fetch upstream
git checkout -b vehicle<n>-dims upstream/main
# ros2_ws/src/mitt_description/config/vehicles/vehicle<n>.yaml 만 수정
#   wheelbase: 0.641          # m  축 중심 간 거리, 줄자, 2026-11-20
#   min_turning_radius: 1.58  # m  calibration.md 1.5, 좌우 중 큰 값
python3 scripts/check_vehicle_dims.py        # 반드시 통과
git add ros2_ws/src/mitt_description/config/vehicles/vehicle<n>.yaml
git commit -m "vehicle<n>: measured <항목들>"
git push origin vehicle<n>-dims
```
GitHub에서 **base: `jrkwon/mrider` main ← compare: `dongguekim-rgb/mrider` vehicle<n>-dims**로 PR을 만듭니다.

- **측정한 값만** 씁니다. TODO는 넣지 않고, 안 잰 항목은 아예 적지 않습니다(그 항목은 추정값이 쓰임).
- 값마다 **어떻게 쟀는지** 주석을 답니다.

---

## 6. 11/30 중간발표 + Merge 2 (팀원 전원 발표)

발표 12분 + 질문 5분입니다.

| 시간 | 내용 | 담당 |
|---|---|---|
| 2분 | 각 track이 맡은 것 | 팀리드 |
| 3분 | Merge 1 결과와 **측정한 숫자**, 실패한 항목과 이유 | E + C |
| 4분 | Merge 2: twin geometry, test, CI (**녹화 영상** 권장) | S + W |
| 3분 | track끼리 아직 받아야 할 것과 **기한** | 각 track 1명 |
| 3분 | 잘 안 되고 있는 것 (솔직하게) | 팀리드 |

- **계획만 말하는 건 좋은 답이 아닙니다.** 결과나 숫자, 아니면 실패한 이유를 보여 줍니다.
- 같은 날 Peer evaluation 1이 있습니다: Contribution, Reliability, Collaboration 1~5점과 "git에 안 보이는 기여" 자유서술. 성적에 반영되는 것은 Week 14 평가이고, Week 12는 점검용입니다.

---

## 7. Track 간 의존관계

```
Chassis ──측정 치수, 회전 반경──▶ Simulation (twin) ──▶ Merge 2
   │                          └─▶ Software (Nav2)
   ▼
Electrical ──firmware, DbwStatus──▶ Software (mitt_hardware, odometry) ──▶ Merge 3, 4
   └──조향 범위, rate, 지연──▶ Simulation (sim과 real 비교)
```

막히면 standup을 기다리지 말고 바로 팀 채팅에 올려 주세요.
