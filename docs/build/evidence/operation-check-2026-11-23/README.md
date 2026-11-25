# 센서 전용 작동 재확인 — 2026-11-23

`ros2_humble`, `vehicle=1`, domain 212, `test_sensor_only_launch.py`를 새로 실행했다.
GUI 없는 자동 시험이며 Gazebo 검증이 아니다.

**결과: FAIL.** 목표 도달 전에 안전 게이트가 `camera_blocked_or_stale`로 정지했다.

- 목표까지 남은 실제 거리: 1.957882 m.
- raw odometry 위치 오차: 0.103489 m.
- 최종 안전 상태: STOPPED, 명령 속도 0.
- 목표 도달을 못 했으므로 후속 카메라 고장 주입 단계는 실행되지 않았다.
- 주행 검사 실패, 프로세스 정상 종료 검사는 통과했다.
- 현재 reason은 카메라 판정 불가/통행 불가/결과 지연을 합쳐 표시하므로 어느 원인인지는 이 기록만으로 확정할 수 없다.
- 이전 재시험 PASS와 별개로 이번 실패를 보존하며 반복 주행 안정성이 확보됐다고 주장하지 않는다.

[JSON](sensor_only.json) · [실행 로그](launch.log) · [XML](test_test_sensor_only_launch.py.xunit.xml)
