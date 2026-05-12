"""
Moveo bridge — where the mock FANUC controller's joint state goes out to the
real BCN3D Moveo arm (or a no-hardware simulation of it).

Reconciled against the actual arm code in
`Robotic Arm/Code/esp32s3_arm_controller/esp32s3_arm_controller.ino` and
`Robotic Arm/Code/Mac GUI/moveo_publisher.py`:

  * The ESP32-S3 runs micro-ROS over USB-CDC serial (agent on the Pi:
    `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200`).
    Node `moveo_esp32`. It SUBSCRIBES to:
       /joint_commands  sensor_msgs/JointState  (BEST_EFFORT)  — `position[0..4]`
                        in RADIANS, order [waist, shoulder, elbow, wrist_roll,
                        wrist_pitch]; the `name` field is ignored. Send the
                        GOAL pose only — the firmware runs its own coordinated
                        sinusoidal ramp; it does NOT want a 50 Hz setpoint stream
                        (each new message resets the ramp).
       /speed_scale     std_msgs/Float32  0.0–1.0 runtime speed multiplier
       /home_cmd        std_msgs/Float32  >=0.5 → zero all joints (open-loop)
       /reboot          std_msgs/Float32  >=0.5 → ESP.restart()
    It does NOT publish /joint_states — the arm is open-loop, so the mock's
    FRC_ReadJointAngles returns commanded state.

  * `moveo_publisher.py` runs persistently on the Pi as a ROS node + TCP server
    on port 9000, accepting JSON lines and republishing to /joint_commands:
       {"position": [j1..j5]}   (radians)        {"speed": 0.0–1.0}
       {"home": true}           {"cartesian": [...]}  {"trajectory": [...]}

So we provide three bridges:
  sim   — no hardware; logs + runtime/moveo_state.json + moveo_trace.csv
  ros2  — direct rclpy publisher to /joint_commands + /speed_scale (run from a
          sourced ROS 2 env that can see the arm's DDS / micro-ROS network)
  pi    — TCP socket to moveo_publisher.py on the Pi (no rclpy needed on this
          host — exactly how the Mac GUI talks to the arm)

FANUC is a 6-axis arm; the Moveo arm here is 5 joints. We map FANUC J1..J5 →
Moveo waist/shoulder/elbow/wrist_roll/wrist_pitch and drop FANUC J6 (flange
roll), with a per-joint scale + clamp into the Moveo's real radian limits (taken
from the arm code). The PROTOCOL stays faithful — FRC_ReadJointAngles echoes the
FANUC angles, including the J2/J3 ground-plane coupling that fanuc_ucl handles
client-side; only the physical arm sees the remapped, clamped values.
"""

from __future__ import annotations

import csv
import json
import math
import os
import socket
import time

# --- joint names / mapping ---------------------------------------------------

# JointState.name the arm code uses (the firmware ignores it, the Pi node sets it)
MOVEO_JS_NAMES = ["j1", "j2", "j3", "j4", "j5"]
# human labels for logs / the viewer (positional, same order)
MOVEO_JOINT_LABELS = ["waist", "shoulder", "elbow", "wrist_roll", "wrist_pitch"]

# (scale, lo_deg, hi_deg) per Moveo joint, fed by FANUC J1..J5 in order.
# Limits derived from the arm code's radian bounds, pulled in a bit for safety:
#   j1 (-2.00,2.40) rad ≈ (-114.6°,137.5°)   j2 (-1.95,1.95) ≈ ±111.7°
#   j3 (-2.20,2.20) ≈ ±126.0°               j4 (-3.14,3.14) ≈ ±180.0°
#   j5 (-1.75,1.75) ≈ ±100.3°
# FANUC joints swing ±180°, so scales <1 keep typical commands in range; the
# clamp catches the rest. Tune these to your build before running on hardware.
MOVEO_JOINT_MAP = [
    (0.55, -110.0, 130.0),   # FANUC J1  → waist
    (0.45,  -95.0,  95.0),   # FANUC J2  → shoulder   (conservative — the risky one)
    (0.55, -115.0, 115.0),   # FANUC J3  → elbow
    (0.70, -175.0, 175.0),   # FANUC J4  → wrist_roll
    (0.55,  -90.0,  90.0),   # FANUC J5  → wrist_pitch
]                            # FANUC J6  → (dropped — Moveo has no flange roll)


def fanuc_to_moveo_deg(fanuc_deg: list[float]) -> list[float]:
    out = []
    for i, (scale, lo, hi) in enumerate(MOVEO_JOINT_MAP):
        v = (fanuc_deg[i] if i < len(fanuc_deg) else 0.0) * scale
        out.append(max(lo, min(hi, v)))
    return out


def fanuc_to_moveo_rad(fanuc_deg: list[float]) -> list[float]:
    return [math.radians(v) for v in fanuc_to_moveo_deg(fanuc_deg)]


# --- bridge interface --------------------------------------------------------

class MoveoBridge:
    """Sink for joint state coming out of the mock FANUC controller.

    Two granularities:
      set_goal()   — called once per FANUC motion with the *final* target; this
                     is what real-arm bridges act on (the firmware ramps itself).
      set_joints() — called for every interpolation step (~50 Hz); the sim uses
                     it to feed the viewer; real-arm bridges ignore it.
    """

    def set_goal(self, fanuc_deg_target: list[float], duration_s: float) -> None:
        pass

    def set_joints(self, fanuc_deg: list[float], moving: bool) -> None:
        pass

    def set_override(self, percent: float) -> None:
        pass

    def close(self) -> None:
        pass


# --- sim (no hardware) -------------------------------------------------------

class SimMoveoBridge(MoveoBridge):
    def __init__(self, runtime_dir: str, verbose: bool = True):
        os.makedirs(runtime_dir, exist_ok=True)
        self.state_path = os.path.join(runtime_dir, "moveo_state.json")
        self.trace_path = os.path.join(runtime_dir, "moveo_trace.csv")
        self.verbose = verbose
        self._t0 = time.monotonic()
        with open(self.trace_path, "w", newline="") as fh:
            csv.writer(fh).writerow(
                ["t_s", "moving"]
                + [f"fanuc_J{i+1}" for i in range(6)]
                + [f"moveo_{n}_deg" for n in MOVEO_JOINT_LABELS])
        self.set_joints([0.0] * 9, moving=False)

    def set_joints(self, fanuc_deg: list[float], moving: bool) -> None:
        moveo_deg = fanuc_to_moveo_deg(fanuc_deg)
        t = round(time.monotonic() - self._t0, 4)
        tmp = self.state_path + ".tmp"
        with open(tmp, "w") as fh:
            json.dump({"t_s": t, "moving": moving,
                       "fanuc_deg": [round(v, 3) for v in fanuc_deg[:6]],
                       "moveo_deg": [round(v, 3) for v in moveo_deg],
                       "moveo_joint_names": MOVEO_JOINT_LABELS}, fh)
        os.replace(tmp, self.state_path)
        with open(self.trace_path, "a", newline="") as fh:
            csv.writer(fh).writerow([t, int(moving)]
                                    + [round(v, 3) for v in fanuc_deg[:6]]
                                    + [round(v, 3) for v in moveo_deg])
        if self.verbose and not moving:
            joined = "  ".join(f"{n}={v:6.1f}°" for n, v in
                               zip(MOVEO_JOINT_LABELS, moveo_deg))
            print(f"   [moveo:sim] {joined}", flush=True)

    def set_override(self, percent: float) -> None:
        if self.verbose:
            print(f"   [moveo:sim] speed_scale → {max(0.0, min(1.0, percent/100)):.2f}",
                  flush=True)


# --- direct ROS 2 publisher --------------------------------------------------

class Ros2MoveoBridge(MoveoBridge):
    """Publishes the goal pose to /joint_commands (sensor_msgs/JointState,
    BEST_EFFORT, 5 positions in radians) and override to /speed_scale (Float32),
    matching esp32s3_arm_controller.ino and moveo_publisher.py exactly.

    Run run_mock.py from a sourced ROS 2 env that can reach the arm's micro-ROS
    network (e.g. on the Pi, or with matching ROS_DOMAIN_ID / FastDDS transport).
    """

    def __init__(self, joint_topic: str = "/joint_commands",
                 speed_topic: str = "/speed_scale",
                 node_name: str = "fanuc_mock_bridge", verbose: bool = True):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float32

        self._rclpy, self._JointState, self._Float32 = rclpy, JointState, Float32
        self.verbose = verbose
        if not rclpy.ok():
            rclpy.init()
        self._node = Node(node_name)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._pub = self._node.create_publisher(JointState, joint_topic, qos)
        self._spub = self._node.create_publisher(Float32, speed_topic, qos)
        self.joint_topic, self.speed_topic = joint_topic, speed_topic
        if verbose:
            print(f"   [moveo:ros2] publishing JointState on {joint_topic}, "
                  f"Float32 on {speed_topic}", flush=True)

    def set_goal(self, fanuc_deg_target: list[float], duration_s: float) -> None:
        rad = fanuc_to_moveo_rad(fanuc_deg_target)
        msg = self._JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = list(MOVEO_JS_NAMES)
        msg.position = [float(v) for v in rad]
        self._pub.publish(msg)
        self._rclpy.spin_once(self._node, timeout_sec=0.0)
        if self.verbose:
            deg = fanuc_to_moveo_deg(fanuc_deg_target)
            print(f"   [moveo:ros2] → {[f'{v:.1f}°' for v in deg]}", flush=True)

    def set_override(self, percent: float) -> None:
        m = self._Float32()
        m.data = float(max(0.0, min(1.0, percent / 100.0)))
        self._spub.publish(m)
        self._rclpy.spin_once(self._node, timeout_sec=0.0)
        if self.verbose:
            print(f"   [moveo:ros2] speed_scale → {m.data:.2f}", flush=True)

    def close(self) -> None:
        try:
            self._node.destroy_node()
        except Exception:
            pass


# --- TCP-to-Pi (moveo_publisher.py) bridge -----------------------------------

class PiSocketMoveoBridge(MoveoBridge):
    """Sends JSON lines to moveo_publisher.py's TCP server on the Pi — same path
    the Mac GUI uses. No rclpy needed on this host.

        {"position": [j1..j5]}   radians, → /joint_commands
        {"speed": 0.0-1.0}       → /speed_scale
        {"home": true}           → /home_cmd
    """

    def __init__(self, host: str, port: int = 9000, verbose: bool = True):
        self.host, self.port, self.verbose = host, port, verbose
        self._sock: socket.socket | None = None
        self._connect()

    def _connect(self) -> None:
        try:
            s = socket.create_connection((self.host, self.port), timeout=3.0)
            s.settimeout(3.0)
            self._sock = s
            if self.verbose:
                print(f"   [moveo:pi] connected to {self.host}:{self.port}", flush=True)
        except OSError as e:
            self._sock = None
            print(f"   [moveo:pi] WARN: can't reach {self.host}:{self.port} ({e}) "
                  f"— is moveo_publisher.py running on the Pi? will retry", flush=True)

    def _send(self, obj: dict) -> None:
        line = (json.dumps(obj) + "\n").encode("utf-8")
        for _ in range(2):
            if self._sock is None:
                self._connect()
            if self._sock is None:
                return
            try:
                self._sock.sendall(line)
                # drain any ack the server sends, non-fatally
                try:
                    self._sock.recv(256)
                except (socket.timeout, OSError):
                    pass
                return
            except OSError:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None
        print("   [moveo:pi] WARN: send failed (Pi unreachable)", flush=True)

    def set_goal(self, fanuc_deg_target: list[float], duration_s: float) -> None:
        rad = fanuc_to_moveo_rad(fanuc_deg_target)
        self._send({"position": [round(v, 5) for v in rad]})
        if self.verbose:
            deg = fanuc_to_moveo_deg(fanuc_deg_target)
            print(f"   [moveo:pi] → position {[f'{v:.1f}°' for v in deg]}", flush=True)

    def set_override(self, percent: float) -> None:
        self._send({"speed": round(max(0.0, min(1.0, percent / 100.0)), 3)})

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass


# --- factory -----------------------------------------------------------------

def make_bridge(kind: str, *, runtime_dir: str, verbose: bool = True,
                ros2_joint_topic: str = "/joint_commands",
                ros2_speed_topic: str = "/speed_scale",
                pi_host: str | None = None, pi_port: int = 9000) -> MoveoBridge:
    kind = (kind or "sim").lower()
    if kind == "sim":
        return SimMoveoBridge(runtime_dir, verbose=verbose)
    if kind == "ros2":
        return Ros2MoveoBridge(joint_topic=ros2_joint_topic,
                               speed_topic=ros2_speed_topic, verbose=verbose)
    if kind == "pi":
        if not pi_host:
            raise ValueError("--moveo pi requires --pi-host (the Pi's hostname/IP)")
        return PiSocketMoveoBridge(pi_host, pi_port, verbose=verbose)
    raise ValueError(f"unknown bridge kind {kind!r} (use sim | ros2 | pi)")
