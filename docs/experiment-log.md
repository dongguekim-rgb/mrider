# 시뮬레이션 시행착오 기록

마지막 갱신: **2026-11-26 KST**. 팀 W·S의 센서 기반 시뮬레이션에서 중요한 실패·수정·재검증을 이 파일에 누적한다.
원본 로그·XML·측정값은 `docs/build/evidence/`에 보존한다. 수업·공식 설계 문서를 대신하는 문서가 아니다.

## 현재 판정

**일부 시험은 성공했지만 전체 시뮬레이션은 아직 성공 상태가 아니다.**
최신 완료 묶음은 `regression-02`이며, 조향 제약·scan 시각 TF 선택·추종 여유를 보완한 뒤 같은 시나리오를 다시 검사한 결과다.
과거 버전에서 성공한 장애물 정지·자동 재개를 현재 버전의 성공으로 간주하지 않는다.

| 이번 실행 | 판정 | 확인한 값·실패 내용 |
|---|---|---|
| 단위 시험 | PASS | 안전 43, 지역 목표 10, 경로 곡률 17, 경로 메시지 9, 차량 overlay 2: **81개** |
| 센서 기본 8 m 주행 | PASS | 목표 도착 오차 `<0.5 m` assertion, 카메라 중단·복구 후 정지 latch 통과. JSON의 0.160576 m는 **고장 주입 후 최종 거리**이며 도착 순간 수치가 아님 |
| 큰 맵 24 m 주행 | PASS | **도착 오차 0.091858 m**, 중간 목표 4개, 충돌 없음, 카메라 고장 latch 통과. 고장 주입 후 최종 거리는 0.207639 m |
| 큰 맵 직진 → 90도 좌회전 → 90도 우회전 | **FAIL** | 첫 `(24,0)` 목표 오차 0.063460 m. 다음 목표는 `no_observed_progress_goal`로 거부. 이번 실행에서는 두 회전 목표 미완료 |
| 작은 맵 좌우 우회·횡단 정지·자동 재개 | **FAIL** | 우회 초반에 controller patience 초과, `planning_or_tracking_failed` 정지. 목표까지 10.046949 m 남아 보행자 정지·재개 단계 미완료 |
| 이전 정답 지도·위치 기반 Navigation 기준 시험 | PASS | 센서 자율주행의 성공 근거와 분리 |
| 위 ROS 시험의 프로세스 정상 종료 | **4 PASS / 1 FAIL** | 큰 맵 회전 시험 종료 중 SLAM `karto::Exception`, exit code −6 |

개별 XML 기준 **91개 중 88개 PASS, 3개 FAIL, 오류·skip 0개**다.
실패 3개는 주행 2개와 종료 검사 1개다.
CTest 묶음 기준으로는 10개 중 8개 통과·2개 실패이며, 이 묶음 수를 개별 테스트 수에 더하지 않는다.
`colcon test` 명령 종료 코드만으로 합격을 판단하지 않는다.

실행 환경은 기존 Docker `ros2_humble:/root/mrider-team`, ROS2 Humble, vehicle 1,
카메라·LiDAR 합성 입력과 실제 slam_toolbox/Nav2를 연결한 **2D 센서 시뮬레이터**다.
GT는 평가 노드만 사용한다. 이번 시험은 RViz 없이 수행했고 시험 프로세스는 종료했다.
공식 Gazebo 물리 twin·실제 카메라/A2M4-R2·Teensy 통합 주행의 통과를 뜻하지 않는다.

근거: [집계 JSON](build/evidence/path-curvature-2026-11-26/regression-02/summary.json),
[빌드 로그](build/evidence/path-curvature-2026-11-26/regression-02/build.log.gz),
[테스트 로그](build/evidence/path-curvature-2026-11-26/regression-02/test.log.gz),
[개별 XML](build/evidence/path-curvature-2026-11-26/regression-02/xml/),
[실행 소스 SHA256](build/evidence/path-curvature-2026-11-26/regression-02/source-sha256.json).
[실행 당시 소스](build/evidence/path-curvature-2026-11-26/regression-02/source.tar.gz)도 보존했다.
[원본 JSON 압축본](build/evidence/path-curvature-2026-11-26/regression-02/reports/)에는 궤적·명령·관측 지도가 포함된다.

직전 [regression-01](build/evidence/path-curvature-2026-11-26/regression-01/summary.json)은 88개 중 86개 통과·2개 실패였다.
당시 24 m 도착 오차는 0.030437 m, 큰 맵 회전 시험은 TF 시각 부족, 작은 맵은 `obstacle_stop` 후 재개 대기 초과로 실패했다.
이번에는 TF 시각 부족이 재현되지 않았지만 목표 선택의 다른 거부 원인과 SLAM 종료 실패가 드러났다. 이전 결과도 그대로 보존한다.

## 기록·갱신 규칙

1. **이후 중요한 실험·실패·수정·회귀 발견 때마다 이 파일을 갱신한다.** 실험을 시작하기 전에 현재 미해결 항목을 확인하고, 종료 후 결과와 증거 링크를 추가한다.
2. 관찰한 사실, 원인 가설, 확인된 원인, 적용한 수정, 재검증 결과를 구분한다. 코드 수정만으로 `해결`이라고 쓰지 않는다.
3. 실패 기록은 삭제하거나 성공으로 덮어쓰지 않는다. 새 결과를 추가하고 맨 위의 현재 판정을 갱신한다.
4. 날짜·환경·차량·시나리오·실행 소스·재현 명령을 남긴다. 미커밋 실행이면 소스 해시와 수정 상태를 보존한다.
5. 도착 순간 오차와 고장 주입 후 최종 거리를 구분한다. 단위 시험, ROS 주행, GUI, Gazebo, 실차, GT 기준 시험의 결과도 구분한다.
6. 제한을 완화하거나 테스트를 바꾸면 이유와 변경 전후를 기록한다. 실패를 감추기 위한 조향 한계·센서 지연 허용치 확대, GT 입력 추가, 무조건적인 재시작으로 통과시키지 않는다.
7. 재현하지 않은 항목은 `미검증`, 원인이 확정되지 않은 항목은 `조사 중`으로 표시한다. README와 시나리오 문서가 서로 다른 최신 상태를 안내하지 않도록 링크를 함께 갱신한다.

## 주요 이력

### E01. 정답 위치로 움직이는 기준 시험과 센서 자율주행의 혼동

- **문제:** 초기 `indoor_demo`·`turns_demo`는 알려진 지도와 모의 정답 위치를 받는 계획·제어 시험이었다. 사용자의 카메라·LiDAR만 사용하는 요구사항을 검증한 것이 아니었다.
- **변경:** `sensor_only=true` 경로에서 world의 제어용 지도·odom TF 발행을 끄고, LiDAR odometry와 온라인 SLAM을 연결했다. 정답 토픽은 `/evaluation/ground_truth`로 분리했다.
- **검증:** 센서 ROS 시험에서 지도·odom 발행자와 GT 구독자 격리를 검사한다. GT 격리 통과와 목표 주행 성공은 별도 판정한다.
- **상태:** 입력 구조 분리 적용. 실센서 정확도 검증은 남음.
- **근거:** [센서 Navigation](sensor-only-navigation.md), [현재 구조](architecture.md).

### E02. 큰 맵의 무반사 LiDAR 값과 잘못된 SLAM 정합 — 2026-11-25

- **증상:** 큰 공간에서 카메라가 시작 영역을 미확인으로 판단해 출발 거부. 이후에는 GOAL 상태에도 실제 목표 오차가 약 22 m인 실패가 발생했다.
- **확인된 원인:** 합성 LiDAR의 유한한 최대 거리 값은 무반사를 뜻했으나, 정합에는 차량과 함께 움직이는 가짜 표면처럼 들어갔다.
- **실패한 시도:** SLAM range threshold 축소와 loop closure 비활성화만으로 해결되지 않았다.
- **변경:** 의미가 확인된 합성 센서에서만 최대 거리까지 자유 공간 근거를 사용한다. SLAM 전용 scan 복사본에서는 무반사·비정상 점을 제외하고, 원본 scan과 좌표·시각을 보존한다.
- **검증·한계:** 이후 장거리 도착 검증을 통과했다. 실제 A2 드라이버의 반환 의미를 확인하지 않고 같은 설정을 적용할 수 없다.
- **근거:** [입력 계약·개발 실패 기록](large-office-navigation.md), [당시 결과](build/evidence/large-office-progress-2026-11-25/verified/summary.json).

### E03. `Failed to compute odom pose` 반복 — 2026-11-25~26

- **증상:** slam_toolbox에서 odom pose 계산 경고가 반복됐다.
- **확인된 원인:** ROS Header 객체 공유로 SLAM용 scan의 `laser` frame이 다른 메시지를 만드는 과정에서 바뀌었다.
- **변경:** Header를 깊은 복사하여 원본 frame·timestamp를 보존했다. TF 지연 한계를 늘리는 방식으로 처리하지 않았다.
- **재검증:** 해당 수정의 네 SLAM 실행에서 경고 0회. 이후 별도로 발견한 scan/TF 콜백 순서 문제(E09)와는 다른 원인이다.
- **근거:** [헤더 수정·검증](sensor-only-navigation.md#slam-tf-fix), 커밋 `618fa0c`.

### E04. 24 m 도착 후 교차로에서 조향 초과 — 2026-11-26

- **최초 관찰:** 24 m 목표 오차 0.145956 m로 도착한 뒤 북쪽 목표에서 `untrackable_curvature` 정지. 오른쪽 회전은 미실행이었다.
- **추가 재현으로 확인한 원인:** SLAM 해상도는 0.05 m인데 costmap 초기값은 0.10 m였다. Smac이 셀 단위로 저장한 회전 반경이 StaticLayer의 해상도 변경 후 실제 길이로는 절반이 됐다. 설정 반경 3.5 m가 경로에서는 약 1.75 m였다.
- **측정:** 재현 후보 곡률 약 0.58447 m⁻¹, 필요 조향 약 0.302 rad. vehicle 1의 모의 물리 한계는 약 0.248561 rad다.
- **변경:** 처음부터 해상도를 일치시키고 설정/실행 중 해상도 불일치를 거부한다. 실제 경로 곡률과 초기 추종 곡률도 검사한다.
- **상태:** 원인 수정 및 단위 검사 통과. 연속 좌·우회전까지 통과한 것으로 간주하지 않는다.
- **근거:** [최초 주행](large-office-navigation.md#live-large-office-2026-11-26), [경로 생성 수정](large-office-navigation.md#path-curvature-fix).

### E05. 평활화가 회전 제약을 다시 깨뜨림 — 2026-11-26

- **관찰:** 해상도를 수정하고 원호 목표를 선택한 뒤에도 평활화된 경로 곡률이 약 0.501575 m⁻¹로 나왔다. 출발 전 검사 한계는 약 0.453515 m⁻¹다.
- **변경:** 센서용 Smac의 단순 평활화를 끄고 전진 Dubins 경로를 유지한다. 설정값뿐 아니라 실제 좌표의 곡률을 검사한다.
- **재검증:** 과도한 곡률·역방향·불연속 경로가 `FollowPath`에 전달되지 않는 검사를 통과했다.
- **회귀:** 평활화를 끈 이후 작은 맵 우회의 추종 여유 문제가 드러났다(E10). 이 변경만으로 전체 성공이라고 결론 내리지 않는다.

### E06. 중간 목표의 위치와 방향 불일치 — 2026-11-26

- **관찰:** XY 거리만 줄이는 목표는 차 옆에 놓이거나, 최종 목표 가까이에서 급격한 도착 방향을 요구했다. 실제 경로 계획 실패와 `original_goal_missed`로 이어졌다.
- **시도:** 큰 방향 전환에만 원호 목표를 적용하고 최종 접근 범위를 8 m로 늘렸으나, 다른 중간 목표의 도착 방향이 여전히 전진 기하와 맞지 않았다.
- **변경:** 모든 중간 목표의 위치와 방향을 같은 전진 원호의 끝 자세로 선택한다. 원호 전체가 관측된 자유 공간 안에 있어야 한다.
- **후속 보완:** 최종 목표가 아직 미확인/막힘 상태이면 중간 원호 끝에서 계획 회전 반경만큼의 최종 접근 거리를 남긴다. 이 보수적인 조건도 E12의 후보 거부 원인을 분리할 때 확인해야 한다.
- **재검증:** 막힌 최종 목표 주변에서도 중간 목표 방향이 해당 원호의 접선인지 검사하는 단위 시험을 추가했다. 최신 24 m 단계별 주행은 통과했다. 큰 맵의 전체 회전 미션은 미완료다.

### E07. 최소 반경 경계와 과도한 여유 반경 — 2026-11-26

- **관찰 1:** 중간 목표를 계획 최소 반경 경계에 정확히 맞추면 격자·방향 반올림으로 약 6 m 원호 대신 26 m 이상의 우회 원이 생성되는 사례가 있었다.
- **시도 1:** 중간 목표 원호 반경에 20% 여유를 주었다. 이번에는 복도 코너 쪽으로 너무 넓게 접근하여 Nav2의 전방 충돌 예측으로 멈췄다. 실제 충돌은 없었다.
- **현재 변경:** 중간 목표 선택 여유를 5%로 조정했다. 물리 조향 한계와 안전 게이트 여유는 그대로다. 큰 맵 계획 반경은 최소 3.0 m, 작은 좌우 우회 시험은 최소 3.5 m다.
- **상태:** 최소 반경 여유만 키운다고 모든 공간에서 성공하지 않음을 확인했다. 차량 제약·장애물 여유·추종을 함께 검증해야 한다.

### E08. 경로 결과와 최신 TF 수신 순서 — 2026-11-26

- **관찰:** 경로 계산 결과가 TF 콜백보다 먼저 처리되어 `path_start_transform_stale`로 출발 전 거부됐다.
- **변경:** PLANNING/정지 상태에서 최대 1초 동안 최신 TF 수신을 기다리도록 했다. TF 나이 0.35초 제한은 유지한다.
- **재검증:** TF가 신선해지면 실행하고, 기한 내 신선해지지 않으면 `FollowPath` 없이 실패하는 단위 검사 통과.
- **한계:** 이 보완은 경로 계산 **이후** 검사다. 다음 항목의 목표 선택 시 scan 시각 TF 문제까지 해결한 것은 아니다.

### E09. 다음 목표 요청 시 scan 시각 TF 부족 — 보완 후 다른 거부 원인 확인

- **regression-01:** 큰 맵 첫 목표 도착 뒤 북쪽 목표를 요청했지만 `Lookup would require extrapolation into the future`로 거부됐다.
- **로그:** 요청 scan 시각 `1795687089.044892`, 최신 TF 시각 `1795687088.944894`. 차이는 약 0.1초다. 차량은 READY/속도 0 상태였고 시험은 5초 대기 후 실패했다.
- **진단:** 최신 scan이 도착한 시점에 그 scan의 odometry/TF 처리가 아직 끝나지 않은 경우다. 좌·우회전을 성공했다고 보고할 수 없다.
- **다음 검증:** 동일 시각의 TF가 준비된 신선한 scan을 선택하거나, 목표 처리를 제한된 시간 동안 대기시킨다. 오래된 scan을 허용하거나 다른 시각 TF를 무조건 대신 사용하지 않는다. 그 뒤 같은 목표 미션을 다시 실행한다.
- **regression-02:** 최근 scan 버퍼에서 나이 0.35초 이하이고 해당 scan 시각의 TF가 준비된 관측을 선택하도록 수정했다. 최신 입력의 누락·잘못된 frame을 과거 입력으로 숨기지 않는 검사까지 통과했다. 이번 큰 맵에서는 TF 외삽 오류 대신 `no_observed_progress_goal`이 발생했다. 다음 목표를 수행하지 못한 문제는 남아 있다.

### E10. 기존 우회·자동 재개 회귀 실패 — 최신 미해결

- **관찰:** 작은 `sensor_turns`에서 고정 장애물에 접근하다 정지했고, 자동 재개 조건이 충족되지 않았다. 이번 정지는 성공해야 할 보행자 횡단 대기와 구분해야 한다.
- **regression-01의 사실:** mission FAILED, safety `obstacle_stop`, 충돌 없음, 최소 이격 약 0.602027 m, 목표까지 9.923040 m 남음. 재개 대기 35초 초과.
- **조사 중:** 해상도·평활화 수정으로 달라진 계획 경로와 RPP의 안쪽 절단, 차체 여유의 상호 작용. 현재 자료만으로 하나의 원인으로 확정하지 않는다.
- **다음 검증:** 계획 경로·추종 명령·실제 궤적을 함께 기록하고 계획 여유를 보완한다. 물리 한계나 정지 조건을 완화하지 않고 기존 횡단 정지·재개·목표 도착·카메라 고장 시험을 모두 다시 통과해야 한다.
- **regression-02:** 계획용 footprint padding을 0.25 m에서 0.40 m로 늘리고 경로·명령·평가 궤적을 기록했다. 최소 이격은 0.734122 m였지만, 우회 초반 Nav2 전방 충돌 예측과 controller patience 초과로 `planning_or_tracking_failed` 정지했다. **여유 확대만으로 해결되지 않았다.** 실제 충돌은 없었고 물리·정지 한계는 유지했다.

### E11. 실행 격리·종료 확인도 실험의 일부 — 2026-11-26

- **관찰:** 한 개발 시험에서 앞선 launch 종료와 다음 동일 domain 시험 시작이 잠깐 겹쳤다. 새 시험은 `manual_arm_required`로 시작에 실패했다.
- **조치:** 해당 실행을 유효한 주행 성능 측정에서 제외하고 실패 원본을 보존했다. 이후 모든 시험 노드의 종료를 확인한 뒤 순차 실행했다.
- **별도 기록:** 개발 실행에서 관측한 SLAM 종료 중 `karto::Exception`이 regression-02의 큰 맵 회전 시험 종료에서도 재현됐다(exit code −6). regression-01의 정상 종료 5회와 구분한다. regression-02는 정상 종료 4개·실패 1개이며, 종료 예외를 무시하거나 합격으로 바꾸지 않았다.
- **원칙:** 테스트별 domain을 분리하고 종료를 확인한다. RViz 반복 재시작을 원인 수정으로 간주하지 않는다.

E04~E11의 개발 원본: [개발 실행 압축본](build/evidence/path-curvature-2026-11-26/development-runs.tar.gz).
폴더명 `raw`, `clean` 등은 개발 단계 이름이며 성공 표시가 아니다.

### E12. 관측된 자유 공간과 다음 회전 목표 — 최신 미해결

- **regression-02 관찰:** 24 m 첫 목표 도착 후 북쪽 목표에서 `no_observed_progress_goal`이 발생했다. 주행기는 READY/속도 0이었으며 새 경로를 실행하지 않았다.
- **확정 범위:** 해당 시점의 관측 지도·scan·차체 여유·전진 원호 조건을 함께 만족하는 후보를 목표 선택기가 반환하지 않았다. 모든 경우에 경로가 물리적으로 없다는 증명은 아니다.
- **조사할 내용:** 관측 지도 경계·미확인 셀·이동 장애물 흔적과 목표 원호 제한이 각각 후보를 어디서 제외하는지 기록해야 한다. 현재 자료만으로 한 원인을 확정하지 않는다.
- **다음 검증:** 후보 거부 사유를 분리하고 관측 갱신/목표 처리 정책을 검증한다. 미확인 영역을 강제로 자유 공간으로 바꾸거나 정답 지도로 후보를 만드는 방식은 사용하지 않는다.

## 다음 통과 조건

- 같은 큰 맵에서 직진·90도 좌회전·90도 우회전의 모든 목표에 도착하고, 각 목표 오차 `<0.5 m`, 방향 오차 `<0.3 rad`, 충돌 없음, 물리 조향 한계 준수를 확인한다.
- 작은 맵의 고정 장애물 우회, 보행자 앞 정지, 제거 후 자동 재개, 목표 도착과 카메라 고장 latch를 다시 통과한다.
- 이번 문제를 수정해도 기존 기본 주행·24 m·입력/TF/조향 검사에 새 실패가 생기지 않아야 한다.
- 실제 센서·Gazebo twin·Teensy DBW는 각각 별도 검증 결과를 남긴다.

## 재현 명령

기존 설명서의 Docker 설치·빌드 준비를 마친 상태에서 실행한다. 같은 domain의 수동 demo는 종료하고 GUI 없이 검사한다.

```bash
docker exec -it ros2_humble bash
cd /root/mrider-team
source ros2_ws/setup_env.sh
source ros2_ws/install-docker/setup.bash
cd ros2_ws
colcon --log-base log-docker build --build-base build-docker --install-base install-docker \
  --symlink-install --packages-select mrider_navigation_sim --cmake-args -DBUILD_TESTING=ON
source install-docker/setup.bash
export MRIDER_TEST_VEHICLE=1
export MRIDER_SIM_REPORT_DIR=/tmp/mrider-regression-report
colcon --log-base log-docker test --build-base build-docker --install-base install-docker \
  --packages-select mrider_navigation_sim --event-handlers console_direct+ \
  --ctest-args -R '^safety$|^local_goal$|^path_feasibility$|^path_messages$|^vehicle_overlay$|test_(sensor_only|large_office|large_office_turns|sensor_turns|navigation)_launch.py' \
  -j1 --output-on-failure
colcon test-result --test-result-base build-docker/mrider_navigation_sim/test_results --verbose
```

이 명령은 현재 소스를 실행한다. 과거 결과와 정확히 비교하려면 링크된 소스 해시도 확인한다.
`test-result`에는 이전 실행 XML이 남아 있을 수 있으므로 이번 실행의 CTest 로그·개별 XML·JSON을 함께 판정한다.

## 새 기록 양식

```text
ID / 날짜(KST) / 제목:
환경·차량·시나리오·소스:
증상과 실제 측정값:
원인 가설 / 확인한 원인:
변경 내용 / 적용하지 않은 시도:
재검증 명령·결과:
원본 증거 링크:
남은 한계와 다음 통과 조건:
```
