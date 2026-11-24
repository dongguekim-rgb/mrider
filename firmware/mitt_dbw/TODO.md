# 펌웨어 남은 할 일

**코드는 다 썼고 빌드도 됩니다. 아직 차에서 한 번도 돌려 보지 않았습니다.**
남은 일은 두 가지입니다. 차에 올려서 단계별로 확인하는 일과, 담당자가 정해야 할 4가지입니다.

이 문서는 차에 펌웨어를 올리고 시험할 Track E 팀원을 위한 것입니다. 위에서부터 순서대로 하세요.

---

## 지금 상태

| 항목 | 상태 |
|---|---|
| 차량 3대용 빌드, 중단 시험용 빌드 | 됩니다 |
| Teensy에 올려서 실행 | **아직 안 했습니다** |
| 노트북과 통신 (micro-ROS) | **아직 안 했습니다** |
| 게인, 보정값 | **전부 임시값입니다** |

---

## 먼저 알아 둘 말

| 말 | 뜻 |
|---|---|
| ESTOP | 차가 멈춘 상태. 켜면 항상 이 상태로 시작합니다 |
| MANUAL_RC | 조종기로 운전하는 상태 |
| AUTONOMOUS | 노트북 명령으로 운전하는 상태 |
| 무장(arm) | ESTOP에서 나와 운전할 수 있게 되는 순간 |
| CH6 | 조종기의 3단 스위치(FS-i6의 SwC). 아래=KILL, 가운데=MANUAL, 위=AUTO |
| 펄스 | Teensy가 모터 드라이버(MDDS30)에 보내는 신호. 1500 µs면 멈춤입니다 |
| 워치독 | 펌웨어가 멈추면 Teensy를 자동으로 재시작하는 타이머 |
| agent | 노트북과 Teensy를 이어 주는 프로그램(micro-ROS agent) |
| Layer A | 조종기 신호를 Teensy가 받아서 조향 각도로 바꾸는 길 |
| Layer B | 조종기 신호가 Teensy를 거치지 않고 모터 드라이버로 바로 가는 길 |
| E4 | 조향 오차 1°를 못 맞출 때 쓰는 대안. 전용 모터 제어기로 바꿉니다 |

---

## 1. 준비 (처음 한 번)

1. **PlatformIO를 공식 설치기로 설치합니다.** VS Code 확장도 됩니다.
   `pip install --user platformio`로 설치하면 빌드가 실패합니다.
   설치 명령은 [04-firmware.md §4.2](../../docs/build/04-firmware.md#42-toolchain-setup)에 있습니다.
2. **ROS를 불러오지 않은 새 터미널을 엽니다.** 빌드는 이 터미널에서만 합니다.
3. **차 번호에 맞는 환경으로 올립니다.** 2번 차라면 다음과 같이 합니다.

   ```bash
   cd firmware/mitt_dbw
   pio run -e vehicle2 -t upload
   ```

   첫 빌드는 몇 분 걸립니다. 다른 차 번호로 올리면 노트북에서 차가 보이지 않습니다.
4. **조종기 수신기의 failsafe를 설정합니다.** 신호가 끊기면 CH6이 KILL로 가게 합니다.

---

## 2. 0단계: 모터 없이 (Teensy와 센서만)

모터 드라이버에는 아무것도 연결하지 않습니다.

1. **agent를 켜고 차가 보이는지 확인합니다.**

   ```bash
   MITT_VEHICLE=2 source ros2_ws/setup_env.sh
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
   ros2 topic echo /mitt/dbw/event        # 다른 터미널에서
   ```

   `session up: vehicle 2, domain 12, time synced`가 보이면 연결된 것입니다.
2. **조향 0점을 잡고 저장합니다.** 바퀴를 똑바로 놓고, 차가 ESTOP일 때 합니다.

   ```bash
   ros2 service call /mitt_dbw/zero_steering std_srvs/srv/Trigger
   ros2 service call /mitt_dbw/save_params std_srvs/srv/Trigger
   ```

3. **조향 눈금을 맞춥니다.** 각도계로 왼쪽 끝과 오른쪽 끝을 잽니다.
   그 값으로 `steer_rad_per_count`를 계산해 넣고 저장합니다
   ([calibration.md §1.2](../../docs/design/calibration.md)).
   숫자는 `3.0`처럼 소수점을 붙여서 넣습니다. `3`은 거부됩니다.
4. **각도 센서 선을 뽑아 봅니다.** `faults`에 1이 켜지고 차가 ESTOP에 머물러야 합니다.
5. **바퀴를 앞으로 굴려 봅니다.** `drive_ticks`가 커져야 합니다.
   작아지면 엔코더 A선과 B선을 바꿉니다.
6. **30분 연결 시험을 합니다.** 아래 5장의 `dbw_link_monitor`를 씁니다.

---

## 3. 1단계: 조향 모터만, 바퀴 띄움 (Merge 1)

**전류 제한이 있는 실험용 전원을 씁니다.** 전류는 모터가 겨우 움직일 만큼만 줍니다.

1. **방향을 확인합니다.** 아래 두 가지가 틀리면 **모터 선을 바꿉니다.** 펌웨어는 고치지 않습니다.
   - RC1 펄스가 1500 µs보다 크면 바퀴가 **오른쪽**으로 돌아야 합니다.
   - RC2 펄스가 1500 µs보다 크면 차가 **앞으로** 가야 합니다.
2. **게인을 맞춥니다.** `steer_kp`만 먼저 올립니다. 그다음 `steer_kd`, 마지막에 `steer_ki`를 조금 넣습니다.
3. **조향 측정을 합니다.** 도구를 먼저 켜고, CH6을 KILL에서 AUTO로 올립니다. 약 75초 걸립니다.

   ```bash
   ros2 run mitt_bench dbw_steer_test --ros-args -p out_dir:=docs/build/records/vehicle2
   ```

   기준은 다음과 같습니다.
   - 정상상태 오차 1° 이하
   - 10° 계단 응답: 0.4초 안에 90% 도달, 넘침 15% 이하
   - ±20° 왕복 추종 오차(RMS) 1.5° 이하
   - ±30°를 명령해도 22.5°에서 멈춤
4. **1° 이하를 못 맞추면 튜닝을 멈추고 E4로 바꿉니다.** "거의 됐다"며 계속 튜닝하면 감점입니다.
5. **멈춤(stall) 감지를 확인합니다.** 측정 중에 바퀴를 손으로 잡습니다.
   0.5초 안에 `faults`에 8이 켜져야 합니다.
6. **명령 끊김을 확인합니다.** 약 0.5초 안에 ESTOP이 되어야 합니다.

   ```bash
   ros2 run mitt_bench dbw_stale_test --ros-args -p out_dir:=docs/build/records/vehicle2
   ```

7. **펌웨어 멈춤 시험을 합니다.** 중단 시험용 빌드를 올립니다.

   ```bash
   MITT_VEHICLE=2 pio run -e hang_test -t upload
   ros2 service call /mitt_dbw/test_hang std_srvs/srv/Trigger
   ```

   모터가 멈출 때까지 걸린 시간을 영상으로 잽니다. 0.2초 안이면 됩니다.
   3초쯤 걸리면 워치독 클록이 틀린 것입니다. 이 경우 `src/hw.cpp`의 `kWdogClockHz`를 고칩니다.
   **시험이 끝나면 차량용 빌드(`vehicle2`)를 다시 올립니다.**
8. **펄스 주기 한계를 잽니다.** 지금은 50 Hz입니다. 더 빠르게 해도 드라이버가 따라오는지 확인합니다.
   결과를 `src/config.h`의 `kPwmHz`와 dbw.md §12에 적습니다.
9. **전원을 끊어 봅니다.** E-stop을 누르면 조향축이 손으로 쉽게 돌아야 합니다.

---

## 4. 2단계: 모터 두 개, 조종기 두 길

1. **Layer A를 확인합니다.**
   - CH6을 MANUAL로 하면 `DbwStatus.mode`가 1이 되어야 합니다.
   - 스틱으로 조향 **각도**를 정할 수 있어야 합니다.
   - AUTO 중에 스틱을 30% 넘게 움직이면 0.2초 안에 MANUAL_RC가 되어야 합니다.
   - 조종기를 끄면 ESTOP이 되어야 합니다.
2. **Layer B를 확인합니다.** Teensy를 리셋 상태로 잡고 CH5 스위치를 올립니다.
   이 시험은 아래 6장의 첫 번째 결정이 나야 할 수 있습니다.

---

## 5. 마지막 확인: 30분 연결 (최종 합격 조건)

이 도구는 듣기만 합니다. 어떤 상태에서 켜도 안전합니다.

```bash
ros2 run mitt_bench dbw_link_monitor --ros-args -p duration_s:=1800.0 \
    -p out_dir:=docs/build/records/vehicle2
```

기준은 다음과 같습니다.
- `/mitt/dbw/status`가 매 분 50 Hz 이상
- USB 끊김 0회
- `/imu/data`가 100 Hz 이상
- 펌웨어 루프가 200 Hz 이상

**측정이 끝나면 기록 파일(YAML, CSV)을 커밋합니다.** 기록이 없으면 통과로 치지 않습니다.
`dbw_steer_test` 기록의 `for_simulation` 값은 Track S에 넘깁니다.
이 값은 조향 범위, 조향 속도 한계, 명령 지연입니다.

---

## 6. 정해야 할 것

담당자가 정해야 하는 일입니다. 정하기 전까지 아래 "지금 펌웨어" 열대로 동작합니다.

| # | 정할 것 | 지금 펌웨어 | 누가 |
|---|---|---|---|
| 1 | **Teensy가 꺼지면 Layer B로 조향할 수 없습니다.** 릴레이 신호선(핀 21)이 풀다운 저항 때문에 꺼짐이 되고, 모터가 원래 제어기로 넘어가기 때문입니다. 배선을 바꿀지, 시험 기준을 바꿀지 정해야 합니다 | 바꾸지 않음 | 팀리드, 교수님 |
| 2 | CH6을 모드 스위치로 쓰고, CH6이 KILL을 벗어날 때 무장합니다 | 이렇게 동작 | Track E |
| 3 | 릴레이는 무장 중일 때와 ESTOP 직후 조향을 가운데로 돌리는 동안만 켭니다 | 이렇게 동작 | Track E |
| 4 | `DbwStatus.faults`의 64번 비트를 E-stop으로 씁니다. 메시지에 이름(`FAULT_ESTOP`)을 붙이려면 upstream에 PR을 보내야 합니다. fork에서는 메시지를 바꾸지 않습니다 | 비트는 이미 씀 | 팀리드 |

---

**다음 할 일:** 1장 준비를 마치고, 2장 0단계의 1번(agent 연결)부터 시작하세요.
자세한 사용법은 [README.md](README.md)에 있습니다.
