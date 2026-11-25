# 방향 A: M2·M3·M4 요구사항과 검증

확인일: 2026-11-16. 이 문서는 구현·시험 계획이다. 아래 실센서·펌웨어·실차 항목은 모두 **NOT RUN**이다.
강의의 설계 기준, 팀의 제안, 실제 측정값을 구분한다. M2/M3의 하드웨어 실습은 원문에도 Draft로 표시되어 있다.

## M4 — 센서 보정과 관측부터

출처: [M4 Perception](https://jrkwon.github.io/mrider/learn/m4-perception/#lecture).
단안 카메라의 풍부한 영상 정보와 LiDAR의 직접 거리 관측을 함께 검증하는 것이 기본이다.
현재 방향 A는 학습 모델을 사용하지 않는다. 영상에 물체가 보이는 것과 자동 장애물 판정은 별개다.

| 시험 | 방법 | 기준·증거 |
|---|---|---|
| 센서 계약 | 실제 scan 각도 간격·유효 거리, 영상 해상도/FOV·주기 측정 | 모델·장착·버전, 측정표. scan 약 10Hz는 전형값이며 모델별 기준 확정 |
| 주기 | IMU와 DBW 상태의 주기·최대 지연 기록 | 강의 기준 IMU ≥100Hz, DBW 상태 ≥50Hz. 평균 주기만으로 timeout 통과 처리하지 않음 |
| 카메라 | global shutter 확인, 체커보드 내부 보정·왜곡 보정 | 강의 목표 재투영 오차 약 0.3px 미만. 실제 오차·보정 해상도·CameraInfo 저장 |
| LiDAR 외부 보정 | 수직 평면 벽의 실측 거리·yaw 및 좌우 부호 비교 | +X 전방, 미러링 없음. 오차 허용치는 측정 장비와 운용 범위에 맞춰 사전 확정 |
| 공동 표적 투영 | scan 취득 시각 TF와 CameraInfo로 영상에 점 투영 | 올바른 optical frame에서 z>0, 영상 내부·유효 range만 사용. 보정값을 임의 픽셀 offset으로 덮지 않음 |
| 사각지대 | 낮은 책, 거리별 의자 다리, 검은 무광, 유리/광택, 소등 장면 | 각 센서의 관측/누락과 이유를 표로 기록. 사람을 충돌 시험 표적으로 사용하지 않음 |
| 보정 오류 주입 | replay 전용 TF에 yaw +5°를 주어 정지·회전 구간 비교 | 정상/오류 overlay와 정합 오차 저장. 실제 차량 URDF를 바꾸거나 중복 TF 발행하지 않음 |

공식 IMU는 Teensy 측에서 발행하는 BNO085 계열 설계다. 노트북용 IMU 드라이버를 중복 실행하지 않는다.
실제 차량 구성이 다르면 인터페이스 소유자와 변경 근거를 기록한다.

팀 구현 계약으로 영상·scan의 상대 시각 오차 상한을 정한다. 지연·TF 부재·CameraInfo 불일치·NaN/Inf는
유효 관측과 구별하여 상태로 내보낸다. overlay는 보정 검증이며 자동 장애물 인식의 합격 증거가 아니다.
정적 extrinsic만으로 이동 중 비동기 센서의 오차가 해결되지 않으므로 회전·이동 bag에서도 검증한다.

### 기록·재생 인계물

ROS2 bag에 `/scan`, `/camera/image_raw`, 실제 CameraInfo 토픽, `/imu/data`, `/mitt/dbw/status`,
`/tf`, `/tf_static`을 기록한다. CameraInfo 추가는 overlay 재현을 위한 팀 요구사항이다.
토픽 이름·형식·QoS·메시지 수·시각 구간, 코드/보정 파일 해시를 함께 저장한다.
재생 시작 전에 static TF와 CameraInfo가 실제 수신되는지 확인하고, 누락 시 해당 시험은 NOT RUN이다.

bag replay는 차량과 분리한 ROS_DOMAIN_ID에서 `use_sim_time`과 단일 `/clock`을 맞추고 DBW 출력은 비활성화한다.
M4의 loop 재생은 시각화용이다. 시계 역행 시 cache를 지우고 정지하는 동작을 검증하기 전 제어 폐루프에 적용하지 않는다.
이 저장소에는 아직 ROS2 overlay/replay launch가 없으며, 구현 후 설치·colcon build/test·실행 명령을 추가해야 한다.

## M2 — 경로 추종과 조향 servo의 경계

출처: [M2 DBW/Steering](https://jrkwon.github.io/mrider/learn/m2-dbw-steering/).
Nav2의 목표를 Teensy가 실제 바퀴 조향각으로 추종해야 한다. 모터 드라이버에 전달되는 effort와 rad 단위 setpoint를 구분한다.

| 시험 | 방법 | 강의 목표·팀 기록 |
|---|---|---|
| 센서/영점 | load-side 절대 센서 전체 travel, 중간점, 재부팅 확인 | single-turn shaft ≤340°, wrap 없음, 선형 보정 잔차 약 0.5° 미만. k가 degree/count이면 rad 변환 명시 |
| 추종 | 측정각과 목표각의 step/sweep 기록 | 정상상태 오차 ≤1°, ±20° sweep RMS ≤1.5°, 10° step 90% 도달 ≤400ms, overshoot ≤15% |
| 세 주기 | host·MCU·driver pulse 각각 계측 | setpoint ≥50Hz, 루프 ≥200Hz. 실제 PWM frame rate는 측정 후 고정 |
| 실패/복구 | sensor fault·limit·stall·전원 손실을 담당자 벤치 절차로 검사 | 명령 echo 금지, fault/측정각/전류/권한 기록. 실차 범위를 넘는 step은 인가하지 않음 |

단계 1에서 ≤1°와 지속 진동 없는 동작을 확보하지 못하면 무한 조정하지 않고 공식 E4 대안 검토로 넘긴다.
부하·노면에서의 재시험은 별도다. 소프트웨어 saturation만으로 조향 기계 한계 검증을 대체하지 않는다.

## M3 — 권한과 고장별 정지 동작

출처: [M3 Teleop/Safety](https://jrkwon.github.io/mrider/learn/m3-teleop-safety/).
권한 순서는 E-stop → motor relay MUX → hardware RC signal MUX → serial RC → laptop autonomy다.
상위 하드웨어 계층을 소프트웨어 명령 선택기로 대체하지 않는다.

| 고장 | 기대 동작·검증 증거 |
|---|---|
| command stale | 500ms watchdog 기준, ESTOP 상태·실측 정지 확인. 유효성 없는 heartbeat 재발행 금지 |
| USB 단절 | MCU 자체 정지와 호스트 stale status에 의한 Nav2 중단을 각각 확인 |
| reset | pulse 중단과 driver signal-loss timeout을 계측 |
| firmware hang | reset 시험과 별도. 마지막 pulse가 유지될 수 있으므로 완료된 루프만 watchdog을 갱신하고 실제 hang에서 reset·정지 확인 |
| hardware E-stop | traction 전원 차단·relay coil 해제·조향 freewheel 확인. 소프트웨어 timeout의 조향 centering과 구분 |
| 복구 | driver 재부팅 전 양 채널 neutral과 RC stick 중앙 확인. 해제만으로 자율 재출발하지 않음 |

RC 상실·logic brownout·조향 한계·각도 센서 단절도 공식 failsafe matrix와 대조한다.
실패 유도는 전류 제한 및 독립 정지 담당자를 갖춘 공식 벤치 절차로만 수행한다.
M3 Stage 0~5 순서를 지키며, reset/hang 결과를 서로의 합격 증거로 재사용하지 않는다.
자율 권한은 Stage 5 이후 허용한다. 초기 지상 시험의 보행 속도 제한·동행 운전자 조건을 유지하고 실제 coast-down 거리를 측정한다.

## 팀 검증 결과의 보관

[기록 양식](real-world-validation-template.md)에 방향 A의 구성과 센서·보정·firmware 버전,
사전 기준, 측정값, 원본 bag/overlay/계측 위치, PASS/FAIL/NOT RUN을 기록한다.
[방향 A 계획](architecture.md)의 지도 주행은 위 선행 시험을 통과한 뒤 수행한다.
이번 작업은 강의 대조와 문서 수정이며 ROS2 노드·firmware 구현이나 하드웨어 합격을 의미하지 않는다.

## 이번 문서 변경의 검사 결과

2026-11-16: Markdown 19개의 로컬 링크 136개, Bash 코드 블록 54개의 구문 검사와
코드 블록 구분자·`git diff --check` 검사를 통과했다. 웹 조회 도구 실패 후 공식 세 페이지를 직접 읽어 대조했다.
Mermaid 렌더링, ROS2 빌드, 센서 bag replay 및 하드웨어 시험은 실행하지 않았다.

## 팀 LiDAR 확정 — 2026-11-16

사용 모델은 RPLiDAR A2M4-R2다(본체 라벨 확인). 공식 tier 권장(G4/S3)과 팀의 A2 선택을 구분한다.
[A2 드라이버·설치·검증 절차](rplidar-a2-setup.md)를 따르며 명목 range/rate를 실측 합격으로 처리하지 않는다.
