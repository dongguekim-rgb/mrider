# Teensy·ROS2 통합 계약

확인일 2026-11-17. 공식 upstream `jrkwon/mrider` 커밋
`d302eb4f399a1bf2f6a7273a0a9210688db5859b`의 메시지와 controller 설정을 확인했다.

## 단일 명령 경로

`Autonomy/Nav2 → 안전·권한 게이트 → ros2_control ackermann_steering_controller → mitt_hardware → /mitt/dbw/command → micro_ros_agent → Teensy`

팀 Pure Pursuit의 Ackermann 명령은 검증 후 차량 제어기의 속도 reference로 변환해야 한다.
현재 `/mrider_sim/reference_preview`는 `geometry_msgs/msg/Twist` 미리보기만 제공한다.
변환은 `linear.x=v`, `angular.z=v*tan(steering_angle)/wheelbase`이며 정지 시 각속도도 0이다.
실측 축거·최종 권한 게이트·feedback 없이 실차 reference로 remap하지 않는다.
공식 설정의 `use_stamped_vel=false`와 controller 입력 토픽/메시지는 설치 버전에서 확인해야 한다.
공식 config의 제원도 TODO measure이므로 팀 실측값으로 승인되지 않았다.

## 공식 메시지와 책임

| 항목 | 계약 |
|---|---|
| mitt_msgs/msg/DbwCommand | stamp, steering_angle(rad), speed(m/s) |
| mitt_msgs/msg/DbwStatus | stamp, steering_angle, steering_setpoint, wheel_speed, drive_ticks, mode, faults |
| 권한 mode | ESTOP=0, MANUAL_RC=1, AUTONOMOUS=2 |
| 주기 | command/status ≥50Hz, Teensy 로컬 조향 루프 ≥200Hz |
| stale | 500ms 기준 firmware 정지와 호스트 status 감시를 각각 시험 |

메시지를 다른 패키지 이름으로 복제하지 않는다. 호스트와 Teensy의 `mitt_msgs` 정의·firmware·micro-ROS 배포판을 맞춘다.
Agent 실행은 전송 통로 제공이며 호환성과 구동 허가를 보증하지 않는다.
실제 wheel/IMU feedback, 시각 동기·QoS, RC 우선·E-stop·재허가를 별도로 확인한다.

## 장치 연결 준비

공식 [micro-ROS Humble 설치/agent 절차](https://micro.ros.org/docs/tutorials/core/first_application_linux/)
및 팀 firmware 담당자의 Teensy 빌드 버전을 기준으로 agent를 준비한다.
`micro_ros_agent`는 기본 모의 패키지 의존성이 아니며 설치 후 아래 명령으로 확인한다.
A2M4-R2와 Teensy의 serial 포트를 혼동하지 않는다. 구동 차단 상태에서 수행한다.

```bash
source /opt/ros/humble/setup.bash
# micro_ros_agent workspace를 사용한다면 해당 install/setup.bash도 source
source ~/mrider-team/ros2_ws/install/setup.bash
ros2 pkg prefix micro_ros_agent
ls -l /dev/serial/by-id/
read -r -p 'Verified Teensy serial device: ' TEENSY_PORT
ros2 launch mrider_autonomy teensy_agent.launch.py serial_port:="$TEENSY_PORT"
```

다른 동일 환경 터미널에서 공식 mitt_msgs를 설치/source한 뒤 검사한다.

```bash
ros2 interface show mitt_msgs/msg/DbwCommand
ros2 interface show mitt_msgs/msg/DbwStatus
ros2 topic info /mitt/dbw/status --verbose
ros2 topic hz /mitt/dbw/status
ros2 topic echo /mitt/dbw/status --once
```

통과 조건: 버전/타입 일치, status 갱신·측정각·속도·mode/fault 확인, 시각과 QoS 확인.
미수신을 정상 상태로 보지 않는다. direct `ros2 topic pub`으로 actuator를 시험하지 않는다.
실차 controller/hardware launch는 차량 보정·권한 계층 확정 후 공식 stack과 통합하며 이번 패키지는 제공하지 않는다.

## 검증 순서

1. 모의 테스트에서 단위·부호·제한·stale 및 `/clock` 정지 감시를 통과한다.
2. 구동 차단 상태에서 장치/firmware/메시지·status 주기·USB 단절을 기록한다.
3. M2 벤치에서 실제 조향·limits·step 응답을 계측한다.
4. M3 권한·E-stop·reset/hang·USB 상실·neutral 복구·수동 재허가를 검사한다.
5. 최종 센서 게이트와 실측 제원 승인 후 제한 구역 저속 시험을 수행한다.

실제 Teensy·센서·차량 검증은 NOT RUN이다.
[결과 양식](real-world-validation-template.md)에 증거를 남긴다.

출처: [공식 DbwCommand](https://github.com/jrkwon/mrider/blob/d302eb4f399a1bf2f6a7273a0a9210688db5859b/ros2_ws/src/mitt_msgs/msg/DbwCommand.msg),
[DbwStatus](https://github.com/jrkwon/mrider/blob/d302eb4f399a1bf2f6a7273a0a9210688db5859b/ros2_ws/src/mitt_msgs/msg/DbwStatus.msg),
[controller 설정](https://github.com/jrkwon/mrider/blob/d302eb4f399a1bf2f6a7273a0a9210688db5859b/ros2_ws/src/mitt_control/config/mitt_controllers.yaml).
