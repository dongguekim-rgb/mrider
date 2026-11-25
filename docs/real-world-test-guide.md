# ROS2 실환경 시험 설명서

현행 환경은 ROS2 Humble이다. [새 PC 빌드](new-pc-setup.md)와 [Teensy 계약](teensy-ros2-integration.md)을 먼저 따른다.
모의 출력은 차량에 연결하지 않는다. 실측 차량 변수·최종 충돌/권한 게이트·공식 hardware integration은 아직 미완료다.

| 단계 | 방법 | 합격 증거 |
|---|---|---|
| A 소프트웨어 | colcon build/test, offline/실토픽 폐루프, Nav2 센서 시뮬레이션 | 결과 XML·커밋·목표 오차·제한·fault 감시 |
| B 센서 단독 | A2M4-R2 scan, camera 보정, wheel/IMU·TF·bag | 거리/각도·시각·유효률·보정 및 사각지대 |
| C 통신 | 구동 차단 상태 Teensy agent·mitt_msgs·status 확인 | firmware/메시지 버전·QoS·주기·stale/USB 단절 |
| D 벤치 | M2 servo 및 M3 권한·독립 정지·reset/hang | 측정각/응답·limits·실측 정지·neutral 복구 |
| E 수동 지상 | 공식 Stage 0~5, 독립 정지 담당자 | RC/E-stop·정지거리·재허가·안전 계층 |
| F 자율 저속 | 선행 합격 후 제한 구역, 장애물/입력 상실 검사 | 횡오차·목표 오차·실제 정지거리·개입·충돌/누락 |

시험 전 속도·조향·footprint·최소 회전반경·감속/정지 거리와 최대 지연을 실측 근거로 승인한다.
`simulation.yaml`의 수치는 승인된 실차 값이 아니다. fail/recovery를 명령값만으로 판정하지 않는다.
정지 요청과 실제 정지의 차이는 독립 계측으로 확인한다.
reset 시험을 firmware hang 합격으로 대체하지 않으며 hardware E-stop은 소프트웨어 정지와 구분한다.
권한 상실/복구 후 자동 재출발 금지 계약은 최종 실차 게이트에서 구현·검증해야 한다.

세부 기준은 [M2/M3/M4 검사표](perception-safety-dbw-validation.md),
[A2 센서 검사](rplidar-a2-setup.md), [방향 A 구현 계획](architecture.md)을 사용한다.
모든 회차는 [실환경 기록 양식](real-world-validation-template.md)에 PASS/FAIL/NOT RUN과 증거를 기록한다.
