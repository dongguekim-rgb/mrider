# Merge 1 전 공식 차량별 Nav2 변경 반영 — 2026-11-22

platinum의 기존 `ros2_humble`에서 실행하는 현재 절차는 [기존 컨테이너 설명서](existing-humble-container.md)를 따른다.

> 저장소 이전(2026-11-22): 이 병합은 **이전 팀 저장소**(`kimdonggue/-Hansla-MRider-Autonomous-Driving`)에서 수행했다.
> "받은 코드와 병합 방식"의 원격 주소와 브랜치 이름은 **그때 실제로 사용한 값**이므로 기록으로 그대로 둔다.
> 반면 "다시 빌드하기"의 명령은 지금 따라 할 절차이므로 현행 checkout(`~/mrider-team`) 기준으로 갱신했다.
> 현행 팀 저장소는 공식 저장소의 **fork**인 `dongguekim-rgb/mrider`이며, 공통 조상이 있으므로
> `--allow-unrelated-histories`가 더 이상 필요하지 않다. 갱신은 `git fetch upstream && git merge upstream/main`으로 한다.

## 받은 코드와 병합 방식

- 공식 원격: `upstream = https://github.com/jrkwon/mrider.git`
- 팀 원격: `origin = https://github.com/kimdonggue/-Hansla-MRider-Autonomous-Driving.git`
- 반영한 공식 main: `2b148bb829a34aef1ab91e9c80d842ae41798a49`.
- 작업 브랜치: `feature/ros1-autonomy-minjun`.

팀 저장소는 공식 저장소와 공통 Git 조상이 없는 독립 저장소였다. `upstream`을 추가하고 두 원격을 fetch한 뒤,
`git merge --allow-unrelated-histories --no-commit --no-ff upstream/main`으로 이력을 연결했다.
README는 팀 안내를 보존하며 공식 변경 링크를 추가했고, `.gitignore`는 양쪽 규칙을 합쳤다.
공식 Apache-2.0 라이선스는 `LICENSE`, 기존 팀 MIT 라이선스는 `LICENSE.team`으로 보존했다.
공식 패키지·차량 overlay·공통 launch·공통 설정·`setup_env.sh`는 upstream 원문과 동일하다.

이후 같은 브랜치에서 공식 변경을 받을 때는 정상적인 fetch/merge가 가능하다.
로컬 변경을 먼저 확인하고 충돌을 해결한 다음 반드시 다시 빌드한다.

```bash
git status --short
git fetch upstream
git merge upstream/main
```

## 공지에서 달라진 인터페이스

- 공통 모듈은 `ros2_ws/src/mitt_description/config/vehicle_config.py`로 이동했다.
  소스 갱신만으로 설치된 패키지 share 경로가 갱신되지는 않으므로 colcon 재빌드가 필요하다.
- 공식 명령은 `ros2 launch mitt_navigation nav2.launch.py vehicle:=<n>`이다.
- `vehicle<n>.yaml`의 실측 차체·바퀴 치수로 footprint를 생성한다. 범퍼 등 추가 돌출부는 자동으로 포함되지 않는다.
- 측정한 `min_turning_radius`의 **1.05배를 cm 단위로 올림**하여 planner의 `minimum_turning_radius`를 생성한다.
- 최신 main에는 footprint의 가장 먼 모서리까지 inflation 영역이 닿도록 하는 보정도 포함됐다.
- `nav2_params.yaml`과 `mitt_dimensions.yaml`은 추정값으로 유지하며, 측정값을 직접 덮어쓰지 않는다.

실차는 **`use_sim_time:=false`를 명시**한다. 이때 차량 번호 1/2/3, 해당 domain(11/12/13),
overlay의 실측 회전 반경이 모두 필요하다. 하나라도 없으면 노드를 띄우지 않고 종료 코드 1을 반환한다.
기본 `use_sim_time:=true`인 twin에는 이 실측 반경 제한이 없다.
실행 중 `/robot_description`과 요청 차량의 차체 불일치 검사는 경고이며 추가 시작 차단 장치는 아니다.

## 다시 빌드하기

### Ubuntu 22.04에 Humble이 설치된 PC

```bash
cd ~/mrider-team
source ros2_ws/setup_env.sh
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix mitt_description
ros2 pkg prefix mitt_navigation
```

### 이전 재빌드 검증용 컨테이너 (기록)

platinum 호스트에는 Humble이 없으며 팀 checkout은 `mrider-ros2-migration`의 `/work`에 연결되어 있다.
기존 개인 Lab용 `ros2_humble`을 바꾸지 않았다.

```bash
docker start mrider-ros2-migration
docker exec -it -e LOGNAME="$(id -un)" mrider-ros2-migration bash
```

컨테이너 안에서:

```bash
cd /work
source ros2_ws/setup_env.sh
cd ros2_ws
colcon --log-base log-docker build --build-base build-docker --install-base install-docker --symlink-install
source install-docker/setup.bash
ros2 pkg prefix mitt_navigation
```

Docker에서는 각 새 터미널에서 공식 `setup_env.sh` 뒤에 `install-docker/setup.bash`도 source한다.
이전 `/work`·호스트 경로가 섞였던 `build/install/log`는
`ros2_ws/legacy-build-20261122-before-upstream/`에 보존했다. 소스는 이동하지 않았다.

## 차량별 준비와 남은 실차 작업

현재 차량 1·2·3 overlay는 모두 비어 있다. 우리 팀 차량 번호와 측정값은 이번 작업에서 임의로 정하지 않았다.
Chassis 담당자가 [Calibration §1.5](design/calibration.md#15-minimum-turning-radius-measured-not-derived)에 따라
좌·우 최대 조향 회전 반경을 각각 측정하고 **큰 값**을 자기 차량 overlay에 기록해야 한다.
측정 대상은 후륜 축 중심인 `base_link`의 회전 궤적이다. 측정 방법과 날짜도 주석으로 남긴다.
이 overlay만 바꾸는 공식 저장소 PR을 준비한다.

```bash
python3 scripts/check_vehicle_dims.py
```

실차 터미널 준비 예시(번호는 반드시 실제 차체 번호 확인):

```bash
cd ~/mrider-team
read -rp '실제 차량 번호(1/2/3): ' vehicle_number
MITT_VEHICLE="$vehicle_number" source ros2_ws/setup_env.sh
# Docker 안에서는 이어서 source ros2_ws/install-docker/setup.bash
ros2 launch mitt_navigation nav2.launch.py vehicle:="$vehicle_number" use_sim_time:=false
```

세 조건 통과는 실차 주행 합격이 아니다. 공식 robot bringup·위치 추정(map→odom)·DBW·권한·비상정지와
센서 검증이 별도로 필요하다. 이번 작업에서는 실차나 Teensy를 연결하지 않았다.
공통 launch와 argument를 바꿔야 하면 공식 저장소 PR로 제안하며 팀에서 다른 인터페이스를 만들지 않는다.

## 팀 시뮬레이션과의 관계

`mrider_navigation_sim`의 `/mrider_sim/*` 환경은 가상 차량 치수로 하는 독립 회귀 시험이다.
공식 `mitt_navigation`과 동시에 실행하거나 모의 명령을 실차에 연결하지 않는다.
공식 차량별 파라미터를 받았다고 팀 시뮬레이션의 치수가 실측으로 바뀐 것은 아니다.
공식 twin 전체는 Gazebo Harmonic/`gz_ros2_control` 등 별도 런타임 준비가 필요하며 이번에는 실행하지 않았다.

## 검증

- 공식 7개 + 팀 2개, **9개 패키지 colcon 재빌드 성공**.
- `check_vehicle_dims.py` PASS: 세 차량 모두 미측정. 추정 planner 반경 1.60m와 footprint가 출력됨.
- `check_packages.py` PASS: 설치 대상 폴더와 Git 추적 파일 존재 확인.
- 설치된 공식 모듈·launch 계약 **6개 PASS**: 공유 모듈 위치, twin 허용, 실차 번호/도메인/실측 누락의 실제 CLI 거부,
  임시 overlay로 footprint·inflation·5% 여유 반경 생성 및 원본 설정 보존.
- 공식 setup_env 실행으로 차량 1/2/3 → domain 11/12/13 확인. Docker overlay에서 `mitt_navigation` 조회 성공.
- 기존 팀 단위·ROS2 통합 회귀 **96개 PASS**. 별도 직선/곡선 offline demo도 PASS.
- **전체 검사가 모두 통과한 것은 아니다.** colcon 전체 집계는 **235 tests, 0 errors, 62 failures, 0 skipped**.
  실패는 upstream 원문 `mitt_description`, `mitt_navigation`, `mitt_bringup`의 pep257/flake8 규칙 위반이다.
  개별 lint 실패 57건과 실패한 실행 wrapper 5건을 합한 수다. 빌드나 팀 주행 기능 시험 실패와 구분한다.
  공통 코드는 수정하지 않았으며, 개선은 공식 저장소 PR로 별도 제안할 대상이다.
- 익명 UID 컨테이너에서 공식 파라미터 생성기의 `getpass.getuser()`가 실패하는 경우를 확인했다.
  실제 사용자명을 `LOGNAME` 환경 변수로 전달해 계약 검사 6개를 재실행하여 통과했다.
- 실차·Teensy·공식 Gazebo Harmonic twin 전체 실행은 미수행. 공식 차량 실측 overlay도 변경하지 않았다.

[전체 빌드·테스트 로그](build/evidence/legacy-results/upstream-2026-11-22/build-test.log),
[차량 계약 검사](build/evidence/legacy-results/upstream-2026-11-22/vehicle-contract.log),
[차량 치수](build/evidence/legacy-results/upstream-2026-11-22/vehicle-dimensions.log),
[패키지 배치](build/evidence/legacy-results/upstream-2026-11-22/package-layout.log),
[도메인 확인](build/evidence/legacy-results/upstream-2026-11-22/environment.log),
[개별 시험 수](build/evidence/legacy-results/upstream-2026-11-22/individual-counts.log),
[패키지별 XML](build/evidence/legacy-results/upstream-2026-11-22/)에 기록했다.

재현 명령(빌드가 완료된 Humble 환경):

```bash
python3 scripts/check_vehicle_dims.py
python3 tools/ros2/test_upstream_vehicle_contract.py
bash tools/ros2/verify.sh
```

두 번째 검사는 설치된 `mitt_description`·`mitt_navigation`을 사용하며, 양성 시험의 측정값은
임시 디렉터리에만 만든다. 실제 `vehicle<n>.yaml`과 기본 Nav2 설정은 바꾸지 않는다.
