# `mitt_bench` — bench measurements for the DBW gates

Three tools that run against the real vehicle, through the micro-ROS agent, and write a record
you can commit. **Wheels off the ground for all three.** Each one prints pass or fail against the
number it is checking, and names the document that sets that number.

> [!WARNING]
> **Written 2026-11-24 and never run against a vehicle.** No tool here has exchanged a
> message with a Teensy yet.

| Tool | What it checks | Source of the number |
|---|---|---|
| `dbw_steer_test` | steady-state error ≤ 1°, 10° step: 90 % in ≤ 400 ms and overshoot ≤ 15 %, ±20° sweep RMS ≤ 1.5°, ±30° commanded is clamped to 22.5° | [`dbw.md` §12](../../../docs/design/dbw.md#12-numeric-interface-contract), Merge 1 |
| `dbw_stale_test` | the laptop goes silent: ESTOP with `FAULT_SETPOINT_STALE` in about 0.5 s | [`safety.md` §2](../../../docs/design/safety.md#2-failsafe-matrix), row 1 |
| `dbw_link_monitor` | 30 min: status ≥ 50 Hz every minute, zero dropouts, IMU ≥ 100 Hz, firmware loop ≥ 200 Hz | [`04-firmware.md` §4.3](../../../docs/build/04-firmware.md#43-stage-0-teensy-alone-no-motor-wired), acceptance gate |

`dbw_steer_test` also writes a `for_simulation` block for Track S: steering range, steering rate
limit, and the latency from a command to the setpoint echo and to motion.

## Running them

The agent is running, the firmware is flashed, and the steering is zeroed
([`firmware/mitt_dbw`](../../../firmware/mitt_dbw/README.md#calibrating)).

```bash
MITT_VEHICLE=2 source ros2_ws/setup_env.sh
R=docs/build/records/vehicle2

# 1. Steering. Start it first, then arm: CH6 KILL -> AUTO. About 75 s.
ros2 run mitt_bench dbw_steer_test --ros-args -p out_dir:=$R

# 2. Staleness. Back to KILL, start it, arm again: CH6 KILL -> AUTO.
ros2 run mitt_bench dbw_stale_test --ros-args -p out_dir:=$R

# 3. The link, 30 minutes. It only listens, so it can run in any mode.
ros2 run mitt_bench dbw_link_monitor --ros-args -p duration_s:=1800.0 -p out_dir:=$R
```

The two tools that command the vehicle **publish 0° and wait for you to arm**. The arm into
AUTONOMOUS succeeds only while a command stream is arriving. Neither tool ever commands speed.
If the vehicle leaves AUTONOMOUS during a run (a stick touched, the kill switch), the run stops.
The record is still written and is marked `aborted`.

**Commit the records**, the CSV beside the YAML. A gate item counts as passed only with its record.

## What is not automated

These need eyes or hands on the vehicle, so no tool does them:

- **Stall bit.** Hold the road wheels during a hold in `dbw_steer_test`, and watch
  `ros2 topic echo /mitt/dbw/status --field faults`. Bit 8 should set within about 0.5 s.
- **Power cut.** The column must turn freely after the E-stop
  ([`safety.md` §4.4](../../../docs/design/safety.md#44-test-procedure-freewheel-on-power-loss)).
- **R/C signal loss and the hang test.** These are timed with a stopwatch or a phone video
  ([`04-firmware.md` §4.4](../../../docs/build/04-firmware.md#44-stage-1-steering-motor-on-the-bench-current-limited)).
- **Layer B**, with the Teensy halted.
