# W·S 팀 공유 준비 및 주행 결과 — 2026-11-23

## 이번 요청으로 확인한 주행

사용자가 켜 둔 `/root/mrider-team`의 실내 demo에서 시작 상태 DISARMED/IDLE와 auto_resume=true를 확인했다.
최초 arm/start를 요청하고 횡단 장애물 앞 정지 → 자동 재개/재계획 → 목표 도달을 검사했다.

| 항목 | 이번 실행 |
|---|---:|
| 목표 위치 오차 | 0.172528 m |
| 최소 차체 이격 거리 | 0.540870 m |
| 충돌 | false |
| 횡단 후 수동 재허가 | 불필요 |
| 판정 | PASS |

[주행 로그](evidence/live-drive-2026-11-23/drive.log),
[궤적·계획·측정 JSON](evidence/live-drive-2026-11-23/indoor_auto.json)을 저장했다.
가상 2D 센서와 정답 odometry의 결과이며 공식 차량 twin이나 실차 주행 결과는 아니다.

## 팀 fork에 공유할 범위

- base: `dongguekim-rgb/mrider`의 `main`.
- head: 같은 fork의 `sw/team-stack-migration`.
- 형식: **Draft PR**. W·S 전체 완료나 Merge 2 합격으로 표시하지 않는다.
- 제목: `W/S: ROS2 모의주행·자동 재개와 기능 검증 이관`.
- 작성자: `junkybox <tim020@naver.com>`.
- upstream PR 대상이 아니다. 공통 interface와 측정 overlay는 변경하지 않았다.
- 미커밋 Dockerfile·gz_ros2_control clone 스크립트는 이 공유 커밋에 포함하지 않는다.

## PR 본문 초안

W·S 팀 코드를 공식 fork로 이관하고 기존 ros2_humble에서 `/root/mrider-team`을 빌드·실행할 수 있도록 준비했습니다.
팀 실내 시뮬레이션은 모의 LiDAR/지도/odometry, Nav2 계획·추종, 장애물 정지 후 조건부 자동 재개를 제공합니다.
실측 차량의 공식 twin이나 실차 stack 완성을 의미하지 않습니다.

검증:

- 9개 패키지 build 성공. 팀 개별 기능 검사 101개의 최종 XML은 실패 0개, 차량별 Nav2 계약 검사 6개 PASS.
- 새 checkout의 첫 실행에서 demo 종료 시 exit -11이 1회 발생했고 해당 시험 재실행에서는 통과했습니다.
  원인 미확정이며 최초 실패 기록을 보존했습니다.
- 전체 colcon 집계에는 공식 원본 lint 실패 62건(wrapper 포함)이 남아 있습니다. CI green으로 표시하지 않습니다.
- 이번 실내 주행은 횡단 후 자동 재개하여 목표 오차 0.173m, 최소 이격 0.541m, 충돌 없이 완료했습니다.
- 증거는 `docs/build/evidence/`에 저장했습니다. 이전 저장소 증거는 `legacy-results/`로 구분했습니다.

남은 W·S 과제:

- 측정 차량 치수의 공식 Gazebo twin 실행·녹화와 sim-real 비교.
- functional test의 GitHub Actions 연결 및 green 확인.
- mitt_hardware, DbwStatus odometry, 카메라 perception, mitt_bench.
- 실측 차량 번호/치수·펌웨어 응답·조향 rate/지연은 C/E와 확인이 필요합니다.

검토 요청:

- W·S 기반 코드 이관 범위와 공식 interface 보존 확인.
- 11/30 Merge 2용 공식 twin·CI와 12/07 Merge 3용 mitt_hardware의 담당/우선순위 합의.

이 PR은 진행 상태와 재현 가능한 근거를 공유하기 위한 Draft이며, merge-ready 판정은 위 미완료 항목과 별도로 결정합니다.
