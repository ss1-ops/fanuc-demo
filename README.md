# FANUC RMI mock controller → BCN3D Moveo

A small project that does two things:

1. **Implements the *server* side of FANUC's Remote Motion Interface (RMI)** — the
   protocol the [`valstad-shipworks/fanuc_ucl`](https://github.com/valstad-shipworks/fanuc_ucl)
   library speaks to an R-30iB controller. The library only ships the *client*;
   there's no way to exercise it without real hardware. This mock fills that gap:
   the unmodified `fanuc_ucl` client connects to it and runs a normal RMI motion
   program against it.

2. **Forwards every motion to a robot arm.** With no hardware it drives a
   simulated arm (logs + a CSV trace + a live stick-figure viewer). Point it at
   ROS 2 and it publishes joint setpoints — so a BCN3D Moveo (or anything on
   `/joint_states`) physically follows a program issued through the FANUC library.

In other words: *I don't have a FANUC, so I made a desktop arm impersonate one
and drove it with the FANUC control library.*


---

## Layout

```
mock_fanuc/
  protocol.py        RMI packet logic — request→response, motion interpolation,
                     speed→duration. Field names mirror fanuc_ucl's serde model
                     (src/rmi/proto/{communication,commands,instructions,
                     member_structs}.rs) so the unmodified client deserialises
                     our responses.
  server.py          TCP server: :16001 FRC_Connect handshake, :16002 the
                     command/instruction stream (strict-FIFO responses).
  moveo_bridge.py    Where joint setpoints go: SimMoveoBridge (no hardware) or
                     Ros2MoveoBridge (sensor_msgs/JointState). Includes the
                     FANUC→Moveo joint scale+clamp map.
run_mock.py          Start the controller.
drive_moveo.py       Demo client built on fanuc_ucl (≈ its own examples/rmi.py).
moveo_viz.py         Optional matplotlib stick-figure of the arm, fed from the
                     sim bridge's state file.
runtime/             (gitignored) sim state + trace written here.
```

## Quick start (no hardware)

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install fanuc_ucl matplotlib        # the library + the optional viewer

# terminal 1 — the "robot controller"
python run_mock.py

# terminal 2 — optional: watch the arm
python moveo_viz.py

# terminal 3 — drive it with the FANUC library
python drive_moveo.py --ip 127.0.0.1
```

Terminal 1 prints every packet in/out — connect handshake, `FRC_Reset` /
`FRC_Abort` / `FRC_Initialize`, `FRC_SetOverRide`, `FRC_SetUTool` /
`FRC_SetUFrame`, a string of `FRC_JointMotionJRep` + `FRC_WaitTime`, and the
final `FRC_ReadJointAngles` — and logs each Moveo setpoint as it interpolates.

You can also run `fanuc_ucl`'s own `examples/rmi.py` unmodified — it hard-codes
`10.0.0.1`, so alias that to loopback first:

```bash
sudo ifconfig lo0 alias 10.0.0.1            # macOS  (Linux: ip addr add 10.0.0.1/32 dev lo)
python run_mock.py --host 10.0.0.1
# then run examples/rmi.py from the fanuc_ucl checkout
sudo ifconfig lo0 -alias 10.0.0.1           # undo when done
```

## Driving the real Moveo

Reconciled against the arm code (`Robotic Arm/Code/esp32s3_arm_controller/…` and
`Robotic Arm/Code/Mac GUI/moveo_publisher.py`). The ESP32-S3 micro-ROS node
`moveo_esp32` subscribes to:

| topic | type | meaning |
|---|---|---|
| `/joint_commands` | `sensor_msgs/JointState`, **BEST_EFFORT** | `position[0..4]` in **radians**, order `[waist, shoulder, elbow, wrist_roll, wrist_pitch]`; `name` ignored. Send the **goal pose only** — the firmware runs its own coordinated sinusoidal ramp; a 50 Hz setpoint stream would fight it. |
| `/speed_scale` | `std_msgs/Float32` 0.0–1.0 | runtime speed multiplier |
| `/home_cmd` | `std_msgs/Float32` ≥0.5 | zero all joint tracking (open-loop) |
| `/reboot` | `std_msgs/Float32` ≥0.5 | `ESP.restart()` |

The arm is **open-loop** (no `/joint_states` feedback), so `FRC_ReadJointAngles`
returns commanded state.

Two ways to reach it:

**(a) via the Pi's `moveo_publisher.py` TCP server (port 9000)** — the same path
the Mac GUI uses; needs no ROS install on this host:
```bash
python run_mock.py --moveo pi --pi-host armpi.local        # or set $MOVEO_PI_HOST
```
Sends `{"position":[j1..j5]}` (radians) and `{"speed":frac}` JSON lines.

**(b) direct rclpy publisher** — run from a sourced ROS 2 env that can see the
arm's micro-ROS network:
```bash
source /opt/ros/jazzy/setup.bash
python run_mock.py --moveo ros2     # JointState on /joint_commands, Float32 on /speed_scale
```

Either way, `FRC_SetOverRide(X)` → `X/100` on `/speed_scale`. For a known start
pose: `ros2 topic pub --once --qos-reliability best_effort /home_cmd std_msgs/msg/Float32 "{data: 1.0}"`.
For visualisation, RViz + your Moveo URDF beats `moveo_viz.py`.

**FANUC→Moveo joint map** (`MOVEO_JOINT_MAP` in `moveo_bridge.py`): FANUC J1..J5
→ waist / shoulder / elbow / wrist_roll / wrist_pitch (FANUC J6 / flange roll is
dropped). Each joint is scaled and clamped into the arm's real radian limits
(from the IK chain in `moveo_publisher.py`): `j1 [-2.00,2.40]`, `j2 [-1.95,1.95]`,
`j3 [-2.20,2.20]`, `j4 [-3.14,3.14]`, `j5 [-1.75,1.75]`. The protocol round-trip
stays faithful — `FRC_ReadJointAngles` echoes the *FANUC* angles (incl. the J2/J3
ground-plane coupling `fanuc_ucl` handles client-side); only the physical arm
sees the remapped, clamped values. **Tune the scales/limits to your build**, and
first run on hardware powered but unloaded/supported.

> The `pi` and `ros2` bridges are written to the arm code's contract but haven't
> been run against hardware from this repo yet — sanity-check with
> `ros2 topic echo --qos-reliability best_effort /joint_commands` on a dry run.

## What's implemented

- Handshake on `:16001` → reports data port `16002` and RMI version 9
  (`fanuc_ucl`'s default `RmiDriverConfig.expected_major_version` is 7, so the
  reported major must be ≥ 7).
- Commands: `FRC_Initialize`, `FRC_Reset`, `FRC_Abort`, `FRC_Pause`,
  `FRC_Continue`, `FRC_SetOverRide`, `FRC_SetUFrameUTool`, `FRC_GetUFrameUTool`,
  `FRC_GetStatus`, `FRC_ReadJointAngles`, `FRC_ReadCartesianPosition`,
  `FRC_ReadTCPSpeed`, `FRC_ReadError`, `FRC_ReadDIN`, `FRC_Read/WriteUFrameData`,
  `FRC_Read/WriteUToolData`, `FRC_Read/WritePositionRegister`, `FRC_WriteDOUT`.
- Instructions: `FRC_SetUTool`, `FRC_SetUFrame`, `FRC_SetPayLoad`, `FRC_WaitTime`
  (sleeps), `FRC_WaitDIN`, `FRC_Call`, and the motion family. `*JRep` motions
  (joint targets) actually move the simulated/real arm with a smootherstep-eased
  interpolation timed from the FANUC speed spec (`mSec`, `Time`, `mmSec` fallback)
  scaled by the active override. Cartesian-target motions are acknowledged
  (protocol-correct) but don't move — no IK in the mock.
- Disconnect: `FRC_Disconnect` is answered and the session closes cleanly.
- Anything not specifically handled gets a permissive `ErrorID: 0` ack
  (`+ SequenceID` for instructions) so new client features don't break the mock.

Single client session at a time on the data port — fine for a demo. Motion and
wait instructions block until done before replying, which matches `fanuc_ucl`'s
blocking RMI mode and keeps the demo deterministic.

## Notes

- On macOS there's one harmless line from the library at connect:
  `ERROR fanuc_ucl::thread_util] Failed to configure thread scheduling: ... only
  supported on Linux`. That's `ThreadConfig`'s realtime-priority hint, which is a
  no-op off Linux — not a problem with the mock.
- This covers **RMI** only. `fanuc_ucl` also implements Stream Motion (UDP,
  ~1 kHz external trajectory streaming), High Speed Position Output, and the
  SNPX-based HMI protocol — see Roadmap.
