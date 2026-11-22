# Docker를 공식 mrider-team으로 전환 — 2026-11-23

## 적용한 상태

- 실행 컨테이너: 기존 `ros2_humble` 유지.
- 팀 checkout: `/root/mrider-team`.
- origin: `https://github.com/dongguekim-rgb/mrider.git`.
- upstream: `https://github.com/jrkwon/mrider.git`.
- 브랜치: `sw/team-stack-migration`.
- 작성자: `junkybox <tim020@naver.com>` (저장소 로컬 설정).
- 빌드·설치: `/root/mrider-team/ros2_ws/build-docker`, `install-docker`.
- 개인 Lab `/root/mrider`와 이전 `/root/mrider-team-work`는 보존했다.

주 컴퓨터의 진행 중인 `tools/ros2/Dockerfile` 수정과 미추적 `clone_gz_ros2_control.sh`는
스테이징·커밋·실행하지 않았다. 이번에는 **커밋된 소스만** 옮겼으며 기존 컨테이너의 패키지 구성을 바꾸지 않았다.
따라서 소스 checkout 전환이 공식 Harmonic twin 환경 구축 완료를 의미하지는 않는다.

## 실행

```bash
docker exec -it -w /root/mrider-team ros2_humble bash
```

컨테이너 안에서:

```bash
source ros2_ws/setup_env.sh
source ros2_ws/install-docker/setup.bash
export ROS_DOMAIN_ID=191
ros2 pkg prefix mrider_navigation_sim
ros2 launch mrider_navigation_sim indoor_demo.launch.py
```

`ros2 pkg prefix`는 반드시 `/root/mrider-team/ros2_ws/install-docker/mrider_navigation_sim`을 반환해야 한다.
이전 checkout을 source했던 터미널 대신 새 터미널에서 위 순서를 사용한다.
최초 arm/start는 [컨테이너 설명서](../existing-humble-container.md)에 따라 별도 터미널에서 요청한다.
기존 시뮬레이션과 새 시뮬레이션을 같은 domain에서 동시에 실행하지 않는다.

## 이후 코드 갱신

호스트의 `/home3/mjk/mrider-team`은 bind mount가 아니다. 수정한 코드를 검토·커밋한 후 전달해야 한다.

```bash
cd ~/mrider-team
bash tools/ros2/sync_existing_container.sh ros2_humble
```

미커밋 작업을 그대로 두고 현재 HEAD만 전달해야 한다면 명시적으로 선택한다.

```bash
bash tools/ros2/sync_existing_container.sh ros2_humble --committed
```

기본 모드는 호스트 미커밋 변경을 거부한다. `--committed`는 호스트 변경을 제외하지만,
컨테이너의 미커밋 변경까지 무시하지는 않는다. origin·브랜치 확인과 fast-forward 조건도 유지한다.
두 보호 조건의 종료 코드 2를 실제 확인했다.

갱신 후에는 컨테이너에서 다시 빌드한다.

```bash
cd /root/mrider-team
source ros2_ws/setup_env.sh
cd ros2_ws
colcon --log-base log-docker build --build-base build-docker --install-base install-docker --symlink-install --cmake-args -DBUILD_TESTING=ON
source install-docker/setup.bash
```

## 이번 검증

- 새 checkout의 공식 7개 + 팀 2개, **9개 패키지 build 성공**.
- `ros2 pkg prefix`로 공식 Nav2·팀 Navigation 모두 `/root/mrider-team/ros2_ws/install-docker`를 사용하는 것을 확인.
- 공식 차량 계약 검사 **6개 PASS**, 차량 치수 검사와 직선·곡선 offline demo PASS.
- 팀 패키지 개별 검사 **101개**의 최종 XML은 실패 0개다(`mrider_autonomy` 45개 + `mrider_navigation_sim` 56개).
- **첫 전체 실행은 완전 통과가 아니었다:** 241 tests, 0 errors, 64 failures, 0 skipped.
  공식 lint 62건에 더해 기본 곡선 demo 종료 시 `autonomy_node` exit -11이 1회 발생했다
  (개별 종료 검사와 실행 wrapper 각 1건). 주행 판정은 통과했고 종료 검사만 실패했다.
- 첫 로그와 XML을 보존한 뒤 해당 demo만 다시 실행했다. 재실행에서는 주행·종료 모두 PASS.
  원인은 확정하지 않았으며 소프트웨어로 해결했다고 주장하지 않는다. 다음 실행에서 종료 오류 재발 여부를 관찰해야 한다.
- 선택 재실행 이후 전체 집계는 **236 tests, 0 errors, 62 failures, 0 skipped**.
  선택 CTest 실행으로 wrapper 목록이 바뀌어 총수가 줄었으며, 팀 개별 검사 101개가 줄어든 것은 아니다.
  남은 62건은 기존 공식 `mitt_bringup`, `mitt_description`, `mitt_navigation`의 pep257/flake8
  개별 위반 57건 + 실패 wrapper 5건이다. 공통 파일은 수정하지 않았다.
- 공식 Gazebo Harmonic twin·실차·`mitt_hardware` 검증은 이번 checkout 전환으로 완료되지 않는다.

[첫 전체 실행](evidence/container-switch-2026-11-23/first-full-run.log),
[첫 종료 오류 XML](evidence/container-switch-2026-11-23/first-autonomy-xml/),
[선택 재실행](evidence/container-switch-2026-11-23/retest.log),
[현재 개별 시험 집계](evidence/container-switch-2026-11-23/individual-counts.json),
[최신 XML](evidence/container-switch-2026-11-23/latest-xml/),
[차량 계약·설치 경로·offline](evidence/container-switch-2026-11-23/contracts-offline.log),
[컨테이너 변경 보호](evidence/container-switch-2026-11-23/sync-dirty-container.log),
[호스트 변경 보호](evidence/container-switch-2026-11-23/sync-dirty-host.log)를 보관했다.

검증 재현(컨테이너, 수동 demo 종료 후):

```bash
cd /root/mrider-team
source ros2_ws/setup_env.sh
source ros2_ws/install-docker/setup.bash
export ROS_DOMAIN_ID=191
python3 tools/ros2/test_upstream_vehicle_contract.py
bash tools/ros2/verify.sh
```

원격 push는 이 전환 과정에 필요하지 않으며, 컨테이너가 사용하는 커밋은 Git bundle로 전달했다.
