# Vehicle 1 실측 overlay 반영 — 2026-11-23

출처: 팀 fork `origin/vehicle1-dims`, commit `badde75b0af6163d3fc8c827272629c395297b96`.
`ros2_ws/src/mitt_description/config/vehicles/vehicle1.yaml`을 원본 그대로 현재 작업 트리에 반영했다.
공식 추정값 파일 `mitt_dimensions.yaml`, 공식 `nav2_params.yaml`, 공통 launch interface는 수정하지 않았다.
현재 작업 브랜치를 전환하거나 기존 W/S 변경을 덮어쓰지 않았다.

| 값 | 반영값 | 원본 측정 근거 |
|---|---:|---|
| wheelbase | 0.533 m | 앞뒤 차축 중심 사이, 줄자, 10/28 |
| track_width | 0.435 m | 앞바퀴 중심 간 거리, 줄자, 10/28 |
| wheel_radius | 0.101 m | 후륜 축 중심~바닥, 장비 적재 전후 동일, 11/23 |
| min_turning_radius | 2.10 m | 수동 조향, 좌측 두 직경의 평균과 우측 직경을 비교, 10/28 |

## 회전 반경 기록 해석

전달 요약의 “4.09/4.30/3.91 중 최댓값 ÷ 2”라면 2.15 m다.
실제 실측 브랜치 주석은 **좌측 평균 `(4.09 + 4.30)/2 = 4.195 m`**와 우측 3.91 m 중
큰 쪽인 좌측의 절반을 사용한다. `4.195/2 = 2.0975 m`를 반올림한 제출값이 **2.10 m**다.
따라서 임의로 2.15 m로 수정하지 않고 원본 2.10 m와 계산 근거를 보존했다.
사용자가 2026-11-23에 **2.10 m가 맞으며 직경 기록을 수정해야 한다**고 확인했다. 반경은 2.10 m로 확정하며, 전달 요약의 “전체 최댓값 ÷ 2” 표현을 좌측 평균을 사용한 원본 계산 근거로 정정한다. 새로운 직경 실측값을 임의로 만들지는 않는다.
펌웨어 조향이 아니므로 공식 절차의 좌우 최대 조향 재측정이 남아 있다.

## 적용 경로

- 공식 `sim.launch.py vehicle:=1` / `nav2.launch.py vehicle:=1`: 공식 생성 규칙 그대로 적용.
- 공식 Nav2 반경은 `ceil_cm(2.10 × 1.05) = 2.21 m`.
- 실차는 `MITT_VEHICLE=1 source ros2_ws/setup_env.sh`, domain **11**. 같은 차량 번호를 sim/real/Nav2에 전달한다.
- 팀 실험 `sensor_only_demo.launch.py vehicle:=1`: 같은 overlay를 읽어 모델·안전 게이트 축거/외곽과 Nav2 footprint에 반영.
- 팀 기본 센서 Nav2는 `max(기존 실험 반경 3.0, 공식 요구 2.21)`로 **3.0 m**를 유지한다.
  좌우 우회·대형 맵 시험은 추정 위치에서의 추종 여유를 위해 **3.5 m** 이상을 사용한다.
- 팀 가상 모델의 등가 조향 한계는 `atan(0.533/2.10)=0.248561 rad`와 기존 한계 중 작은 값.
  이는 수동 측정 반경에 맞춘 운동학적 제약이며 **실차 조향각 캘리브레이션 값이 아니다**.
- 팀 기본 모의 속도 0.5 m/s, 좌우 우회·대형 맵의 0.35 m/s는 실험 제한이며 실측 차량 성능값이 아니다.
- `track_width`·`wheel_radius`는 공식 URDF/컨트롤러에 반영된다. 팀 평면 자전거 모델에는 개별 바퀴 회전 동역학이 없으므로
  두 값은 차체 외곽 산정에만 사용한다. 차체 길이·폭·wheel_width 등 미측정값은 추정치가 남는다.
- 기존 GT 기준 `indoor_demo`·`turns_demo`는 이전 가상 제원을 유지한다. 실측 반영 실험과 혼동하지 않는다.

## Docker 재현

```bash
docker exec -it ros2_humble bash
cd /root/mrider-team
source ros2_ws/setup_env.sh
cd ros2_ws
colcon --log-base log-docker build --build-base build-docker --install-base install-docker \
  --symlink-install --packages-select mitt_description mrider_navigation_sim --cmake-args -DBUILD_TESTING=ON
source install-docker/setup.bash
cd ..
python3 scripts/check_vehicle_dims.py
python3 tools/ros2/test_upstream_vehicle_contract.py
```

하드웨어와 분리된 팀 센서 전용 시뮬레이션만 실행할 때:

```bash
export ROS_DOMAIN_ID=192
ros2 launch mrider_navigation_sim sensor_only_demo.launch.py vehicle:=1
```

위 domain 192는 모의 `/mrider_sim/*` 실행용이며 실차 domain 규칙의 변경이 아니다.
공식 차량 launch는 domain 11 설정을 사용한다. 실제 차량 연결·주행 명령은 이번 작업에서 수행하지 않았다.

## 검증

- Docker: `mitt_description`, `mrider_navigation_sim` 재빌드 성공.
- 치수 검사: vehicle 1 **4 measured**.
- 공식 계약 검사 **6개 PASS**. 미측정 반경 거부 검사는 실제 vehicle1이 미측정이라는 가정 대신 임시 빈 overlay fixture로 검사하도록 수정.
- xacro `vehicle:=1` → URDF 생성 및 역파싱: 축거 0.533, 윤거 0.435, 반경 0.101 m 일치.
- URDF body와 vehicle1 overlay 비교 mismatch 없음.
- footprint: `[[0.76, 0.28], [0.76, -0.28], [-0.23, -0.28], [-0.23, 0.28]]`.
- 공식 Gazebo twin 전체 시연·실차 성능·펌웨어 조향 재측정은 별도 검증 대상이다.


검증 산출물: [치수 검사](evidence/vehicle1-2026-11-23/dimensions.log),
[계약 검사](evidence/vehicle1-2026-11-23/contracts.log),
[생성 URDF](evidence/vehicle1-2026-11-23/vehicle1.urdf),
[파생값 JSON](evidence/vehicle1-2026-11-23/generated-values.json).

실측 제원 주행 첫 실행은 `cannot start: nav2_not_ready`로 실패했다.
Nav2 lifecycle은 active였지만 임무 노드 준비 조건을 통과하지 못했고 차량은 출발하지 않았다.
[첫 주행 기록](evidence/vehicle1-2026-11-23/sensor_only.json),
[로그](evidence/vehicle1-2026-11-23/sensor-test.log)를 보존했다. 이는 실측값 검사를 통과했다는 것과 별개다.

동일 코드·설정 재실행에서는 **주행 검사 1개 + 정상 종료 검사 1개 PASS**.
`vehicle=1`, 센서 전용 입력에서 목표 도달 및 실제 목표 오차 <0.5 m 조건을 통과했고,
후속 카메라 중단 주입으로 정지·복구 후 정지 유지도 통과했다.
고장 검사까지 끝난 최종 위치의 목표 거리 **0.113906 m**, raw odometry 오차 **0.172975 m**.
이는 최초 GOAL 순간 오차와 구분한다. 정답은 검사 노드의 평가에만 사용한다.
[재시험 JSON](evidence/vehicle1-2026-11-23/retry/sensor_only.json),
[재시험 로그](evidence/vehicle1-2026-11-23/retry/launch.log),
[재시험 XML](evidence/vehicle1-2026-11-23/retry/test_test_sensor_only_launch.py.xunit.xml).
최초 준비 상태 실패의 원인은 미확정이며, 재실행 통과를 근본 수정으로 주장하지 않는다.

재시험 명령 (컨테이너의 ros2_ws, 위 환경 source 후):

```bash
export MRIDER_TEST_VEHICLE=1
export MRIDER_SIM_REPORT_DIR=/root/mrider-team/docs/build/evidence/vehicle1-2026-11-23/retry
colcon --log-base log-docker test --build-base build-docker --install-base install-docker \
  --packages-select mrider_navigation_sim --event-handlers console_direct+ \
  --ctest-args -R test_sensor_only_launch --output-on-failure
```

위 기록은 2026-11-23의 기본 주행 시험이다. **2026-11-25에는 같은 vehicle 1 overlay로
좌·우 우회·횡단 정지·자동 재개와 대형 맵 24 m 주행까지 통과**했다.
[현재 결과·재현 명령·남은 검증](../large-office-navigation.md#validation-2026-11-25)을 확인한다.
