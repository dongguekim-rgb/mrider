# W·S 역할 및 기존 Docker 반영 점검 — 2026-11-23

## 기준과 범위

사용자가 우선 기준으로 지정한 `/home3/mjk/Downloads/README (1).md`를 읽었다.
본문은 이 팀 fork의 [.github/README.md](../../.github/README.md)와 앞뒤 공백을 제외하고 일치한다.
첨부본 SHA256: `0a075f33828e144bdd23e745af61adca6fc4401daf8fb17971118d7bd691a0e0`.
담당은 W(Software & Autonomy)와 S(Simulation & Validation)다.
이번 작업은 반영 여부와 역할 적합성 점검이며, 문서 속 push·PR·실차 실행 명령은 실행하지 않았다.

## 가장 중요한 환경 차이

| 항목 | 확인한 상태 |
|---|---|
| 공식 팀 fork | `https://github.com/dongguekim-rgb/mrider.git` |
| 호스트 checkout | `/home3/mjk/mrider-team`, `sw/team-stack-migration`, 점검 시작 HEAD `f4f6b2b` |
| 실행 중 컨테이너 | `ros2_humble` |
| Docker 팀 checkout | `/root/mrider-team-work`, HEAD `4834f2d1`, origin은 이전 `kimdonggue/-Hansla-MRider-Autonomous-Driving` |
| Docker의 새 fork 경로 | `/root/mrider-team` 없음 |
| 기존 mount | 개인 Lab `/home3/mjk/mrider` → `/root/mrider`, X11 소켓 |

**새 fork의 코드가 Docker에 반영됐다고 볼 수 없다.** 호스트의 새 동기화 도구는
`/root/mrider-team`과 올바른 팀 URL을 지정하지만 아직 그 checkout은 없다.
팀 코드 일부가 이전 저장소에서 실행되는 것과, 채점 대상 fork의 커밋을 검증하는 것은 구분해야 한다.

호스트에는 `tools/ros2/Dockerfile` 수정과 `tools/ros2/clone_gz_ros2_control.sh` 미추적 파일이 있다.
이번 점검에서는 이를 수정·커밋·실행하지 않았다. Dockerfile은 작성만으로 기존 컨테이너에 적용되지 않는다.
소스 clone 스크립트도 아직 commit SHA를 고정하지 않은 `humble` branch clone이므로 재현성 보완이 필요하다.

## W 요구사항 대조

| 요구사항 | 판정 | 근거와 남은 일 |
|---|---|---|
| 차량 측정 반경 +5%로 공식 twin에서 Nav2 검증 | 부분 | 임시 overlay 파라미터 생성·시작 차단 시험은 존재. 실측 overlay 기반 공식 twin 주행 증거는 없음 |
| S와 functional test·CI | 부분 | 팀 2D Navigation의 실제 ROS 토픽 시험은 있음. 공식 twin/실차 연결과 CI green은 별개 |
| `mitt_hardware` ros2_control plugin | 미구현 | 소스 패키지 없음, 컨테이너 ament 조회 실패. W 최우선 산출물이 아직 빠짐 |
| `DbwStatus` 기반 odometry | 미구현 | 해당 입력을 처리하는 odometry 구현 없음. 모의 정답 odometry는 대체물이 아님 |
| RViz·카메라 perception | 부분 | RViz 설정은 존재. 카메라 기반 perception 구현은 없음 |

## S 요구사항 대조

| 요구사항 | 판정 | 근거와 남은 일 |
|---|---|---|
| 실측 `vehicle<n>.yaml`로 공식 `sim.launch.py` 실행·녹화 | 미완료 | 차량 번호 미확정, overlay 미측정. Docker에서 `gz_ros2_control` 조회 실패 |
| functional tests | 부분 완료 | 경로 추종·장애물·센서 고장·자동 재개 시험이 있으므로 현재 팀 코드가 전부 linter인 것은 아님. 공식 twin controller 활성화·측정 URDF·조향 clamp 검증은 추가 필요 |
| ROS build + colcon test CI green | 미완료 | 현재 workspace workflow는 build/URDF 확인이며 `colcon test` 단계 없음. tools workflow의 pytest는 scripts/tests용으로 ROS 기능 시험이 아님. 이 fork에서 요구 functional CI가 통과한 증거를 확인하지 못함 |
| sim-real 오차 10% 이내 또는 차이·이유 | 미측정 | 실제 조향 범위·rate·wheelbase·명령 지연 비교 자료 없음. 임의 숫자로 채우면 안 됨 |
| 실제 복도와 같은 parametric world | 부분 | 가상 2D 사무실과 이동 장애물은 있음. 실제 복도 실측 및 공식 Gazebo world와 일치 검증은 없음 |
| `mitt_bench` acceptance 측정 도구 | 미구현 | 소스 패키지 없음, 컨테이너 ament 조회 실패 |

기존 자동 재개 시험은 이전 작업 저장소의 `docs/results/auto-resume-2026-11-22/`에 기록되어 있다.
이전 기록은 Navigation 개별 54개와 추가 보호 2개 통과다. 이번 점검에서 재실행한 수치가 아니다.
**새 fork에는 그 결과 디렉터리가 이관되어 있지 않다.** 이전 전체 검사에는 공식 lint 실패도 있었으므로
기능 일부 통과를 전체 CI green으로 표현하면 안 된다.
첨부 규칙에 맞춰 필요한 로그·그래프·녹화를 이 fork의 `docs/build/` 아래에 정리하고 커밋해야 한다.

## 공통 인터페이스와 작성자

현재 호스트 checkout의 `mitt_msgs`, `mitt_description/config`, 공식 sim/real/nav2 launch와
`setup_env.sh`를 로컬 `upstream/main`과 대조했으며 차이가 없었다.
측정되지 않은 차량값을 기록하거나 공통 interface를 바꾸지 않았다.

사용자 요청대로 팀 checkout들과 기존 Docker 팀 checkout의 **저장소 로컬** Git 설정을
`user.name=junkybox`, `user.email=tim020@naver.com`으로 맞췄다.
전역 설정과 개인 Lab은 변경하지 않았고, 과거 커밋의 author를 재작성하지 않았다.
새 fork의 코드 이관 커밋 `f4f6b2b`는 이미 이 작성자 정보로 기록되어 있다.

## 일정에 맞춘 우선순위

1. 새 fork 기준 Docker checkout·빌드를 확정한다. 기존 Lab을 건드리지 않고 `/root/mrider-team`을 사용한다.
   미커밋 Dockerfile/clone 스크립트는 기존 작업자 변경을 보존하며 검토 후 적용한다.
2. **11/30 Merge 2:** C의 실측값·차량 번호와 E의 조향 범위/rate/지연 측정을 받아 공식 twin을 실행·녹화한다.
   미측정 항목은 미측정으로 표시한다. 기존 가상 2D 데모만으로 Merge 2 완료로 처리하지 않는다.
3. 공식 twin functional test와 GitHub Actions `colcon test` 단계를 연결한다.
   공통 코드 lint 개선이 필요하면 첨부 규칙대로 팀리드에게 알리고 upstream PR 경로로 진행한다.
4. **지금부터 W의 `mitt_hardware`를 시작**한다. `DbwCommand/DbwStatus` 인터페이스와 ros2_control plugin을
   mock 통신으로 먼저 검증하고, E의 펌웨어와 함께 **12/07 Merge 3**를 준비한다.
5. `DbwStatus` odometry·카메라 perception·`mitt_bench` 측정 도구를 준비하여 **12/14 Merge 4**로 연결한다.

11/23 Merge 1의 주 시연은 E+C다. W+S의 핵심 단기 책임은 11/30에 보일 측정 twin·functional CI 증거이며,
동시에 W의 실차 plugin 착수를 미루지 않는 것이다. 팀 연락·push·PR은 이번 점검에서 수행하지 않았다.

## 확인 명령

```bash
git -C ~/mrider-team remote -v
git -C ~/mrider-team status --short
git -C ~/mrider-team config user.name
git -C ~/mrider-team config user.email
docker exec ros2_humble git -C /root/mrider-team-work remote -v
docker exec ros2_humble git -C /root/mrider-team-work log -1 --oneline
```

컨테이너에서는 Humble과 현재 checkout의 install을 source한 뒤 다음을 확인했다.

```bash
ros2 pkg prefix mitt_navigation
ros2 pkg prefix mrider_navigation_sim
ros2 pkg prefix mitt_hardware
ros2 pkg prefix mitt_bench
ros2 pkg prefix gz_ros2_control
```

앞의 두 항목은 이전 팀 install 경로를 반환하고, 뒤 세 항목은 현재 overlay에서 조회되지 않았다.
