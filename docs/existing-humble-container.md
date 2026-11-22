# 기존 ros2_humble 컨테이너에서 팀 프로젝트 실행

2026-11-22 업데이트: 실내 demo는 [장애물 자동 재개](automatic-obstacle-resume.md)가 기본이다. 아래 수동 재허가 절차는 `auto_resume:=false` 또는 센서·명령 오류로 정지한 경우에 사용한다.

2026-11-23 공식 fork 경로 전환을 완료했다. platinum에서는 기존 `ros2_humble`을 팀 작업 실행 환경으로 사용한다.
Humble을 호스트에 새로 설치할 필요가 없다. 기존 개인 Lab 작업과 컨테이너는 유지한다.

| 위치 | 용도 | 연결 방식 |
|---|---|---|
| 호스트 `~/mrider-team` | IDE에서 편집·커밋하는 팀 저장소 | 호스트 Git checkout |
| 컨테이너 `/root/mrider-team` | 팀 빌드·실행 | 별도 Git checkout, 명시적으로 동기화 |
| 컨테이너 `/root/mrider` | 기존 개인 Lab | 호스트 개인 Lab의 기존 bind mount, 수정하지 않음 |

컨테이너의 팀 checkout은 **bind mount가 아니다**. 호스트 파일을 저장만 하면 반영되는 것이 아니며,
아래 스크립트는 커밋한 코드만 전달한다. 컨테이너를 삭제하면 별도 checkout도 삭제되므로 코드 편집과 보관은 호스트에서 한다.
기존 `mrider-ros2-migration`은 이전 검증 환경으로 남겨 두었고 여기서는 사용하지 않는다.

## 1. 코드 갱신 — 호스트 터미널

호스트의 팀 변경을 검토·커밋한 후 실행한다. 원격 push나 GitHub 다운로드는 수행하지 않는다.

```bash
cd ~/mrider-team
bash tools/ros2/sync_existing_container.sh ros2_humble
```

현재 브랜치의 Git bundle을 전달하며, 컨테이너의 origin이 팀 저장소이고 브랜치가 같으며 변경 사항이 없을 때만
fast-forward로 갱신한다. 기본 모드에서는 호스트나 컨테이너에 미커밋 변경이 있으면 중단한다.
호스트에서 진행 중인 작업을 보존하고 **커밋된 HEAD만** 전달하려면 다음처럼 명시한다.
컨테이너의 미커밋 변경은 이 옵션으로도 덮어쓰지 않는다.

```bash
bash tools/ros2/sync_existing_container.sh ros2_humble --committed
```

`--committed`는 미커밋 Dockerfile·소스 등을 제외한다. 그 변경까지 컨테이너에 적용됐다고 해석하지 않는다.
컨테이너 Git의 작성자는 동기화 시 호스트 저장소의 user.name/user.email을 따른다.
 변경을 덮어쓰거나 개인 Lab을 복사하지 않는다.

## 2. 빌드 — 컨테이너 터미널

```bash
docker exec -it ros2_humble bash
```

컨테이너 안에서:

```bash
cd /root/mrider-team
source ros2_ws/setup_env.sh
export ROS_DOMAIN_ID=191
cd ros2_ws
colcon --log-base log-docker build --build-base build-docker --install-base install-docker --symlink-install --cmake-args -DBUILD_TESTING=ON
source install-docker/setup.bash
ros2 pkg prefix mitt_navigation
ros2 pkg prefix mrider_navigation_sim
```

이 경로의 `/root/mrider-team/ros2_ws/install-docker/...`가 출력되어야 한다.
호스트 `/work`·`/home3/mjk`의 빌드/설치 폴더를 복사해서 사용하지 않는다.
기존 컨테이너에 없던 `ros-humble-ackermann-msgs`, `ros-humble-gazebo-msgs`를 추가했다.
기존 Nav2·robot_localization·ROS–Gazebo 연동 패키지는 재설치하지 않았다.

## 3. 팀 실내 시뮬레이션

위 빌드 터미널에서:

```bash
ros2 launch mrider_navigation_sim indoor_demo.launch.py
```

화면이 없는 연결에서는 `rviz:=false`를 붙인다. X11 소켓 마운트는 기존 컨테이너에 있지만,
현재 접속의 DISPLAY/X 인증·GPU 렌더링까지 검증한 것은 아니다.

두 번째 호스트 터미널에서 같은 컨테이너로 들어간 후:

```bash
docker exec -it ros2_humble bash
```

컨테이너 안에서:

```bash
cd /root/mrider-team
source ros2_ws/setup_env.sh
source ros2_ws/install-docker/setup.bash
export ROS_DOMAIN_ID=191
ros2 service call /mrider_sim/arm std_srvs/srv/Trigger '{}'
ros2 service call /mrider_sim/start std_srvs/srv/Trigger '{}'
```

Nav2 초기화 완료 뒤 요청하고 각 서비스 `success: true`를 확인한다.
진행·정지·재허가 절차는 [실내 설명서](indoor-simulation-guide.md)를 따른다.
모든 시뮬레이션 터미널은 같은 domain 191을 사용한다. 기존 Lab과 다른 domain으로 분리하며 `/mrider_sim/*`만 사용한다.
실차는 domain 191을 쓰지 않고, [공식 차량별 안내](upstream-merge-2026-11-22.md)의 `MITT_VEHICLE` 설정을 따른다.

## 4. 검증

수동 demo를 종료한 후 컨테이너의 팀 저장소 루트에서 실행한다.

```bash
cd /root/mrider-team
source ros2_ws/setup_env.sh
source ros2_ws/install-docker/setup.bash
export ROS_DOMAIN_ID=191
python3 scripts/check_vehicle_dims.py
python3 tools/ros2/test_upstream_vehicle_contract.py
bash tools/ros2/verify.sh
```

2026-11-23에 공식 fork의 `/root/mrider-team`에서 9개 패키지를 새로 빌드했다.
차량 계약 6개가 통과했고, 팀 개별 검사 101개의 최종 XML은 실패 0개다.
첫 실행의 demo 종료 오류 1회 및 재실행 통과, 남은 공식 lint 실패는
[전환 검증 기록](build/docker-team-switch-2026-11-23.md)에 구분해 남겼다. 전체 CI green을 의미하지 않는다.

`gz_ros2_control`은 현재 환경에서 조회되지 않았다. 따라서 공식 Gazebo twin 전체가 준비됐다고 간주하지 않는다.
이번 작업은 팀 실내 시뮬레이션과 공식 차량별 Nav2 설정/시작 조건 검증이며, 공식 Harmonic twin 전체와 실차는 별도 검증 대상이다.
