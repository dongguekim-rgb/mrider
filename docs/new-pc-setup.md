# 새 팀원 PC — ROS2 Humble 설치·빌드

platinum의 기존 `ros2_humble`에서 실행하는 현재 절차는 [기존 컨테이너 설명서](existing-humble-container.md)를 따른다.

대상: Ubuntu 22.04, ROS2 Humble, Python 3. ROS1 Noetic/Foxy 환경을 source하지 않은 새 터미널을 사용한다.
Humble은 [공식 deb 설치 절차](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)로 설치한다.
`source`는 설치 명령이 아니다. `/opt/ros/humble/setup.bash`가 없으면 아래 빌드를 진행하지 않는다.
먼저 `cat /etc/os-release`와 `ls /opt/ros`로 실행 중인 환경을 확인한다.
Linux Mint 20.2 / Ubuntu 20.04 호스트에서는 이 Ubuntu 22.04용 설치 절차를 그대로 실행하지 않는다.
Docker 없이 실행하려면 Ubuntu 22.04가 설치된 PC 또는 별도 OS 환경에 Humble을 먼저 설치한다.

이후 아래 명령은 팀 저장소 checkout을 새로 준비하는 경우다. 기존 checkout은 로컬 변경부터 확인한다.

```bash
sudo apt-get install git python3-colcon-common-extensions python3-rosdep python3-pytest python3-scipy python3-opencv \
  ros-humble-ros-base ros-humble-ackermann-msgs ros-humble-rviz2 ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-launch-testing-ament-cmake ros-humble-launch-testing-ros
source /opt/ros/humble/setup.bash
git clone --branch sw/software-autonomy https://github.com/dongguekim-rgb/mrider.git ~/mrider-team
git -C ~/mrider-team remote add upstream https://github.com/jrkwon/mrider.git
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
cd ~/mrider-team/ros2_ws
rosdep install --from-paths src/mrider_autonomy src/mrider_navigation_sim src/mitt_description \
  --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --packages-up-to mrider_navigation_sim --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 pkg prefix mrider_autonomy
colcon test --packages-select mrider_autonomy mrider_navigation_sim --executor sequential --ctest-args -j1 --output-on-failure
colcon test-result --test-result-base build/mrider_autonomy/test_results --verbose
colcon test-result --test-result-base build/mrider_navigation_sim/test_results --verbose
```

각 터미널에서 `/opt/ros/humble/setup.bash`와 팀 `ros2_ws/install/setup.bash`를 source한다.
`mrider_navigation_sim`에는 SLAM 종료와 Nav2의 오래된 costmap 대기를 정리하는 C++ 실행 파일도 포함된다. `package.xml`의 `rclcpp`·`slam_toolbox`·`nav2_controller`·Boost·TBB 의존성은 위 `rosdep install`로 설치하고, 수정 뒤에는 colcon 빌드를 다시 한다.
`catkin_make`/`catkin build`는 현행 workspace에 사용하지 않는다. 다른 PC의 build/install 디렉터리를 복사하지 않는다.
이 문서의 clone은 해당 커밋이 원격에 존재할 때 재현된다. 전달받은 checkout이면 clone을 생략하고 커밋 해시를 기록한다.

위 명령은 팀 센서 시험과 그 의존 패키지를 빌드한다. 공식 Gazebo twin 전체 설치·빌드는 [원본 절차](run-the-twin.md)를 따른다.
현재 브랜치·커밋은 `git branch --show-current`, `git rev-parse HEAD`로 기록한다.

실행: [카메라·LiDAR 센서 주행](sensor-only-navigation.md), [대형 맵·좌우회전](large-office-navigation.md).
초기 제어기의 비교 시험은 [Pure Pursuit 기준 시험](simulation-test-guide.md)이다. 센서 SDK와 micro-ROS agent는
[A2M4-R2 설명서](rplidar-a2-setup.md), [Teensy 통합](teensy-ros2-integration.md)에 따라 별도 설치한다.
센서와 agent 없이도 모의 데모·테스트를 실행할 수 있다. GPU·모델 가중치는 필요하지 않다.

## Docker 검증 대안

호스트 OS가 다르면 저장소 루트에서 전용 이미지로 빌드/시험한다. 장치·호스트 네트워크를 공유하지 않는다.

```bash
docker build -f tools/ros2/Dockerfile -t mrider-humble-test .
docker run --rm -e ROS_DOMAIN_ID=187 -e ROS_LOG_DIR=/tmp/mrider-logs \
  -e LOGNAME="$(id -un)" \
  -u "$(id -u):$(id -g)" -v "$PWD:/work" -w /work mrider-humble-test \
  bash tools/ros2/verify.sh
```

`verify.sh`는 Docker 내부에서 `build-docker/install-docker/log-docker`를 사용하고,
네이티브 환경에서는 `build/install/log`를 사용한다. 컨테이너의 후속 실행 터미널은
`source /work/ros2_ws/install-docker/setup.bash`를 사용한다.
호스트에서 이 컨테이너용 setup을 source하지 않는다. 설치 결과에는 절대 경로가 들어 있다.

### 기존 Docker 산출물을 호스트에서 빌드한 경우

`CMakeCache.txt ... /work/... is different than ... /home/...` 오류는 서로 다른 경로의 빌드 결과를 재사용한 것이다.
`ros2: command not found` 또는 Humble setup 파일 누락이 함께 있으면, 먼저 **Humble 설치 환경**부터 준비해야 한다.
`rosdep update`는 사용자 의존성 데이터베이스를 갱신하며 Humble 자체를 설치하지 않는다.
초기화 순서는 위 설치 블록의 `sudo rosdep init`(최초 1회) → `rosdep update`이다.

Humble이 준비된 Ubuntu 22.04 네이티브 환경에서 기존 산출물을 보존하며 새로 빌드하려면 다음과 같이 백업한다.
소스 디렉터리 `src`는 이동하지 않는다. 기존 ROS 환경을 source하지 않은 새 터미널에서 실행한다.

```bash
cd ~/mrider-team/ros2_ws
backup_dir="legacy-build-$(date +%Y%m%d-%H%M%S)"
mkdir "$backup_dir"
for directory in build install log; do
  if [ -e "$directory" ]; then mv "$directory" "$backup_dir/"; fi
done
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src/mrider_autonomy src/mrider_navigation_sim src/mitt_description \
  --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --packages-up-to mrider_navigation_sim --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 pkg prefix mrider_navigation_sim
```

백업 폴더는 로컬 생성물이며 커밋하지 않는다.
GUI는 데스크톱 PC의 네이티브 실행을 기준으로 한다. headless 테스트 통과는 GPU/OpenGL·디스플레이 연결의 합격을 의미하지 않는다.

## 빌드 경로 분리 검증 (2026-11-17)

Humble이 없는 Linux Mint 20.2 호스트에서 `verify.sh`가 설명과 함께 exit 2로 중단되는 것을 확인했다.
Docker의 새 `build-docker/install-docker` 경로에서 두 패키지 빌드와 전체 회귀 검증을 실행했고,
**110 tests, 0 errors, 0 failures, 0 skipped**였다. 직선·곡선 offline demo도 완료했다.
[이번 실행 출력](build/evidence/legacy-results/indoor/build-isolation.log)을 참조한다. 호스트 네이티브 Humble 실행은 설치되어 있지 않아 수행하지 않았다.

공식 차량 파라미터 생성기는 임시 파일명에 사용자명을 사용한다. 호스트 UID로 컨테이너를 실행할 때
해당 UID가 컨테이너 `/etc/passwd`에 없으면 `getpass.getuser()`가 실패할 수 있으므로 위와 같이
실제 사용자명을 `LOGNAME`으로 전달한다. 공식 업데이트 후 재빌드·차량별 실행은
[차량별 설정과 검증](build/vehicle1-measurements-2026-11-23.md)을 따른다.
