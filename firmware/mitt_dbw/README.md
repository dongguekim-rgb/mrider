# `mitt_dbw` — the vehicle firmware

The Teensy 4.1 drive-by-wire controller: steering position loop, throttle shaping, i-BUS RC
override, the safety state machine, the watchdog, and the micro-ROS link that carries
`DbwCommand`, `DbwStatus` and `/imu/data`.
Spec: [`dbw.md` §10](../../docs/design/dbw.md#10-teensy-firmware-interface) ·
[`safety.md`](../../docs/design/safety.md) · bring-up: [`04-firmware.md`](../../docs/build/04-firmware.md).

> [!WARNING]
> **Built, never run on hardware** (2026-11-24).
>
> The vehicle environments compile and link. **Nothing here has
> driven a motor, talked to an agent, or read a sensor.** Every gain and calibration default is
> a placeholder. Bring it up in the order of [`04-firmware.md`](../../docs/build/04-firmware.md),
> wheels off, on a current-limited supply.
>
> **What is left, step by step, in Korean: [`TODO.md`](TODO.md).**

## Build and flash

**PlatformIO has to be installed the standard way** — the
[installer script](https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html)
or the VS Code extension. Both create `~/.platformio/penv`, and the micro-ROS library build
looks for exactly that folder. `pip install --user platformio` does not create it, and the build
fails with `Can't open .../penv/bin/activate`.

```bash
# In a terminal WITHOUT ROS sourced — the micro-ROS build runs its own colcon.
cd firmware/mitt_dbw
pio run -e vehicle2 -t upload      # the number on the vehicle
```

- **The first build of each environment takes several minutes.** It compiles micro-ROS Humble
  from source with `mitt_msgs` inside it (`extra_packages/mitt_msgs` links to
  `ros2_ws/src/mitt_msgs`). Later builds reuse it.
- **After changing a `.msg` file:** `pio run -e vehicle2 -t clean_microros`, then build again.
- **Ubuntu 22.04 or newer.** The pinned platform's `teensy_size` tool needs glibc 2.34; on 20.04
  the link succeeds and the size check fails.

| Environment | What it is |
|---|---|
| `vehicle1` · `vehicle2` · `vehicle3` | The vehicle firmware. Differs only in the ROS domain: 10 + N |
| `hang_test` | Adds the `/mitt_dbw/test_hang` service for the Stage 1 hang test. `MITT_VEHICLE=2 pio run -e hang_test -t upload`. **Flash the vehicle environment back afterwards** |

## Talk to it

```bash
MITT_VEHICLE=2 source ros2_ws/setup_env.sh
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

| Name | Type | |
|---|---|---|
| `/mitt/dbw/command` | `mitt_msgs/DbwCommand` | in. Best effort. Stale after 500 ms |
| `/mitt/dbw/status` | `mitt_msgs/DbwStatus` | out. Best effort, 100 Hz |
| `/imu/data` | `sensor_msgs/Imu` | out. Best effort, about 200 Hz, `frame_id: imu_link`. Silent while the IMU is faulty |
| `/mitt/dbw/event` | `std_msgs/String` | out. Reliable. Every mode change, every refused arm, and every 5 s a line with the measured loop rate |
| `/mitt_dbw/zero_steering` | `std_srvs/Trigger` | sets `steer_zero_counts` to the current raw angle. ESTOP only |
| `/mitt_dbw/save_params` | `std_srvs/Trigger` | writes every parameter to EEPROM. ESTOP only |

**Status and IMU are best effort.** A laptop node must subscribe with `SensorDataQoS`. A
reliable subscriber matches nothing and receives nothing, with no error. `ros2 topic echo`
and `ros2 topic hz` adapt by themselves.

`ros2 topic echo /mitt/dbw/event` is the first thing to open when something does not happen.
It says *why*: `arm refused: sticks not centred`, `ESTOP: RC link lost`, and so on.

## The transmitter

| Channel | Use | Fixed by |
|---|---|---|
| CH1 | steering stick | wiring — also the signal MUX `SLAVE` input |
| CH2 | throttle stick, **self-centring** | wiring — also the signal MUX `SLAVE` input |
| CH5 | signal MUX `SEL`: above about 1.7 ms hands the driver to the receiver | wiring |
| **CH6** | **mode switch, 3 positions: KILL / MANUAL / AUTO** | **this firmware** — `SwC` on an FS-i6 |

> [!IMPORTANT]
> **Set the receiver failsafe so CH6 goes to KILL.** Whether the FS-iA6B stops sending i-BUS
> frames on signal loss, or keeps sending the last values, depends on its failsafe setting. A
> firmware timeout covers the first case. Only the failsafe value covers the second.

### Arming

The vehicle boots in `ESTOP`, with the relay MUX released (STOCK). **The only way out is to
move CH6 out of KILL**, and only if all of these hold *at that moment*:

1. E-stop released, and released for at least 1 s (the driver boots and calibrates).
2. The steering has been zeroed (`steer_zero_counts` is not −1).
3. The angle sensor reads, and the reading is plausible.
4. Both sticks are centred.
5. For AUTO: `/mitt/dbw/command` is arriving.

A refused arm is dropped, not queued. To try again, go back to KILL first. The reason appears on
`/mitt/dbw/event`.

| From | Event | To |
|---|---|---|
| any armed mode | E-stop pressed, CH6 to KILL, RC lost for 100 ms, or angle-sensor fault | `ESTOP` |
| `AUTONOMOUS` | no `DbwCommand` for 500 ms | `ESTOP` |
| `AUTONOMOUS` | CH6 to MANUAL, **or either stick moved past 30 %** | `MANUAL_RC` |
| `MANUAL_RC` | CH6 to AUTO (an edge, not a position) | `AUTONOMOUS` |

**What ESTOP does.** Throttle goes to neutral at once. If the steering motor still has power, the
loop centres the wheels for up to 2 s, then both lines go to neutral and the relay MUX releases
to STOCK. **While the E-stop sense is low**, the loop does not run at all: both lines output
exactly 1.5 ms, as [`04-firmware.md` §4.3](../../docs/build/04-firmware.md#43-stage-0-teensy-alone-no-motor-wired)
requires.

**The relay MUX (pin 21)** is energised while armed, and during that centring. Otherwise it is
released. So every ESTOP ends in STOCK, as [`safety.md` §1.1](../../docs/design/safety.md) has
every abort do.

## Two wiring rules, and one sign to check

Layer B passes CH1 and CH2 to the driver untouched. So the Teensy has to mean by a pulse what the
transmitter means by it:

- **`RC1`: a pulse above 1500 µs turns the road wheels right.**
- **`RC2`: a pulse above 1500 µs drives forward.**
- **Drive encoder: counts rise when the vehicle rolls forward** (Stage 0 gate).

Check all three at Stage 1 with a small fixed effort. **If one is backwards, fix the wire:**
swap the motor leads at the driver output, or swap the encoder's A and B. Do not flip a sign in
firmware. A firmware sign would make Layer A correct and leave Layer B backwards, and Layer B is
the path that works when this firmware does not. If a transmitter stick is backwards, reverse that
channel on the transmitter. That one change fixes both layers.

## Calibrating

Parameters are ROS parameters on `/mitt_dbw`. Set them with `ros2 param set`, then save them to
EEPROM. **Write a double as `3.0`, not `3`.** An integer is rejected as a type mismatch.

```bash
# Wheels straight, E-stop released, vehicle in ESTOP:
ros2 service call /mitt_dbw/zero_steering std_srvs/srv/Trigger
ros2 param set /mitt_dbw steer_rad_per_count 0.00087     # calibration.md 1.2, signed
ros2 param set /mitt_dbw drive_m_per_tick 0.0161          # calibration.md 2.1, roll-out
ros2 service call /mitt_dbw/save_params std_srvs/srv/Trigger
ros2 param dump /mitt_dbw > config/calibration/vehicle2_dbw.yaml   # keep a record
```

**Calibration parameters** change what an angle or a tick means. They are accepted only in
`ESTOP` with the relay released: `steer_zero_counts`, `steer_rad_per_count`, `steer_limit_rad`,
`steer_stop_rad`, `drive_m_per_tick`.
**Tuning parameters** are accepted at any time, within fixed ranges: `steer_kp/ki/kd`,
`steer_i_max`, `steer_deadband_rad`, `steer_effort_max`, `drive_effort_per_mps`,
`drive_effort_max`, `speed_cap_mps`, `rc_speed_max_mps`, `accel_effort_per_s`,
`decel_effort_per_s`, `reversal_speed_mps`.

The defaults are in [`lib/dbw_core/src/dbw_types.h`](lib/dbw_core/src/dbw_types.h). **Every
default that describes the vehicle is a placeholder.** The safety constants (500 ms staleness,
the 1 s E-stop hold-off, the stall thresholds) are in
[`dbw_limits.h`](lib/dbw_core/src/dbw_limits.h). They are not parameters. Changing one is a
reviewed commit.

## What the firmware gives each bring-up gate

| Gate item | How to see it |
|---|---|
| Loop ≥ 200 Hz, measured | `/mitt/dbw/event`, every 5 s: `loop 250.0 Hz (period … us, exec max … us, overruns …)` |
| ±22.5° clamp | `DbwStatus.steering_setpoint` never exceeds `steer_limit_rad`, whatever is commanded |
| Mechanical-limit clamp | past `steer_stop_rad`: effort towards centre only, `faults & 4` |
| Stall | effort ≥ 40 % with under 0.5° of motion for 500 ms: `faults & 8`, then 1 s of back-off towards centre |
| Setpoint staleness | `faults & 2`; in AUTO, ESTOP after 500 ms |
| Angle-sensor fault | `faults & 1`, ESTOP, no centring |
| E-stop sense | `faults & 64` while low and for 1 s after release |
| Watchdog / hang | `hang_test` build: `ros2 service call /mitt_dbw/test_hang std_srvs/srv/Trigger`. On reboot, `/mitt/dbw/event` says `boot: after a WATCHDOG reset` |
| Time sync | `/mitt/dbw/event` at connect: `time synced (…)`, or `TIME SYNC FAILED` |

## How it is put together

```
src/main.cpp        setup order: watchdog → neutral pulses → EEPROM → control tick → ROS, IMU
src/control.cpp     250 Hz timer interrupt: sense → Supervisor::Step → pulses, relay → feed watchdog
src/ros_link.cpp    micro-ROS session, topics, services, parameters      (main loop)
src/imu.cpp         BNO085 on Wire1                                       (main loop)
src/hw.cpp          AS5600, PWM, relay gate, RTWDOG
src/persist.cpp     parameters in EEPROM with a CRC
lib/dbw_core/       no Arduino: state machine, loop, throttle, i-BUS, sensors
```

**The steering lives in the interrupt, on its own I²C bus.** The main loop can block, because a
USB write or a BNO085 reset takes time, and the steering never waits for it. The watchdog is fed
only at the end of a completed control tick. A hang in the tick therefore resets the board.
Then the pulses stop, and the driver's own signal-loss timeout stops the motors
([`safety.md` row 8](../../docs/design/safety.md#2-failsafe-matrix)).

## Decisions made here that the design documents do not make

Each of these needs a yes, or a correction, from whoever owns the design:

1. **micro_ros_platformio, not micro_ros_arduino.** micro_ros_arduino is precompiled and cannot
   include `mitt_msgs` without a Docker rebuild. micro_ros_platformio comes from the same upstream
   and builds the same Humble sources. It is pinned to commit `cfee17f`.
   [`dbw.md` §9](../../docs/design/dbw.md#9-teensy-41-firmware-platform-and-version-pinning)
   was updated to match on 2026-11-24.
2. **CH6 is the mode switch, and arming is CH6 leaving KILL.** No document assigned a channel or
   defined "RC arm".
3. **The relay gate is high only while armed or centring.** No document said when the firmware
   should energise it.
4. **Bit 64 of `DbwStatus.faults` is the E-stop**, because
   [`04-firmware.md` §4.3](../../docs/build/04-firmware.md#43-stage-0-teensy-alone-no-motor-wired)
   asks for the E-stop in the status and the message had no field for it. The constant
   `FAULT_ESTOP = 64` belongs in upstream `mitt_msgs`. A team fork does not change `mitt_msgs`,
   so the firmware sets the bit whether the constant is there or not. The wire format is the
   same `uint16` either way.
5. **The IMU publishes the game rotation vector (no magnetometer)**, to match `ekf_indoor.yaml`.
6. **Throttle is open loop**: `effort = speed × drive_effort_per_mps`. The design specifies no
   speed loop.
7. **`FAULT_OVER_CURRENT` is never set.** The MDDS30 reports nothing in R/C mode.

## Open, and found while writing this

- **Layer B and a dead Teensy.** The relay board's gate comes from pin 21 with a pull-down. A
  Teensy that is held in reset or unplugged therefore releases the relay MUX to STOCK, and the
  motors are no longer connected to the MDDS30. But the Stage 2 test in
  [`04-firmware.md` §4.5](../../docs/build/04-firmware.md#45-stage-2-both-channels-and-both-override-layers)
  expects *"steering responds to the sticks with the Teensy dead"* through the signal MUX. Both
  cannot be true with this wiring.
- **The watchdog's clock.** The code assumes the RTWDOG runs from the 32 kHz LPO. The Stage 1
  hang test measures the real period. A stop about 30 times slower than 100 ms means the
  clock assumption is wrong.
- **The servo frame rate** stays at 50 Hz until Stage 1 measures what the MDDS30 tracks
  (`cfg::kPwmHz`, [`dbw.md` §4](../../docs/design/dbw.md#4-adr-motor-driver-control-mode-independent-rc-teensy-as-both-masters)).
