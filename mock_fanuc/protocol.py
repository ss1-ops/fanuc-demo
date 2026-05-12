"""
RMI protocol logic for the mock FANUC controller.

This is the *server* side of FANUC's Remote Motion Interface (the "R632"
option) over TCP. The wire format is line-delimited JSON ("<json>\\r\\n").
The authoritative spec used here is the serde model in
`valstad-shipworks/fanuc_ucl` — see `src/rmi/proto/{communication,commands,
instructions,member_structs}.rs`. Field names below match those `#[serde(rename
= ...)]` attributes exactly so the unmodified client library can deserialise our
responses.

Connection handshake:
  1. Client TCP-connects to port 16001 and sends {"Communication":"FRC_Connect"}
  2. We reply {"Communication":"FRC_Connect","ErrorID":0,"PortNumber":16002,
     "MajorVersion":M,"MinorVersion":m} and close the handshake socket.
     NB: fanuc_ucl's default RmiDriverConfig.expected_major_version == 7, so M
     must be >= 7 or the driver rejects the connection.
  3. Client TCP-connects to PortNumber (16002) and from then on exchanges
     Command / Instruction / Communication packets, one response per request,
     strictly FIFO (the client matches responses off a queue).

Two packet "channels" on 16002:
  - Command   : immediate controller queries/settings  ({"Command": "...", ...})
  - Instruction: motion-program steps, carry a SequenceID ({"Instruction": "...",
                 "SequenceID": n, ...}). We treat motion/wait instructions as
                 blocking — we perform the move (or sleep) and only then reply,
                 which matches fanuc_ucl's blocking RMI mode and keeps the demo
                 deterministic.
"""

from __future__ import annotations

import time
import math
import json
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

# --- protocol constants ------------------------------------------------------

HANDSHAKE_PORT = 16001
DATA_PORT = 16002
RMI_MAJOR_VERSION = 9   # >= 7 so fanuc_ucl's default config accepts us
RMI_MINOR_VERSION = 0
LINE_TERMINATOR = b"\r\n"

JOINT_KEYS = ("J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9")


def joints_to_wire(joints: list[float]) -> dict[str, float]:
    """list[9] -> {"J1":..,...,"J9":..}  (the JointAngles serde model)."""
    out = list(joints) + [0.0] * (9 - len(joints))
    return {k: round(float(v), 4) for k, v in zip(JOINT_KEYS, out)}


def wire_to_joints(d: dict[str, Any]) -> list[float]:
    """{"J1":..,...} -> list[9]."""
    return [float(d.get(k, 0.0)) for k in JOINT_KEYS]


# --- robot state -------------------------------------------------------------

@dataclass
class RobotState:
    # Joint angles in FANUC "FanucDeg" wire convention (J3 measured vs the
    # J2-coupled frame). fanuc_ucl's JointFormat.convert_from() handles the
    # AbsDeg<->FanucDeg j3 += j2 / j3 -= j2 fix-up on the client side, so we
    # just store and echo back exactly what we received.
    joints: list[float] = field(default_factory=lambda: [0.0] * 9)
    uframe: int = 0
    utool: int = 0
    override: int = 100         # percent
    payload: int = 0
    _seq: int = 0               # fallback SequenceID counter
    _tt: int = 0                # TimeTag counter (ms-ish)

    def next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def time_tag(self) -> int:
        self._tt += 1
        return (int(time.monotonic() * 1000) & 0x7FFFFFFF) or self._tt


@dataclass
class HandlerResult:
    response: dict[str, Any]
    close_after: bool = False


# --- helpers -----------------------------------------------------------------

def _seq_of(req: dict, state: RobotState) -> int:
    s = req.get("SequenceID")
    return int(s) if s is not None else state.next_seq()


def _move_duration_s(speed_type: str, speed: float, joint_delta_deg: float,
                     override_pct: int) -> float:
    """Best-effort mapping from a FANUC speed spec to a wall-clock duration."""
    st = (speed_type or "").lower()
    if st in ("msec",):                       # SpeedType.MilliSeconds
        base = max(speed, 1) / 1000.0
    elif st in ("time",):                     # SpeedType.TimeSec (0.1 s units)
        base = max(speed, 1) * 0.1
    elif st in ("mmsec", "inchmin"):          # cartesian speed; no IK here
        # rough joint-space fallback: ~60 deg/s nominal
        base = max(joint_delta_deg / 60.0, 0.25)
    else:
        base = 1.0
    ov = max(override_pct, 1) / 100.0
    return max(base / ov, 0.05)


# --- packet dispatch ---------------------------------------------------------

def handle_packet(req: dict[str, Any], state: RobotState, bridge,
                  log: Callable[[str], None]) -> HandlerResult:
    """Map one decoded request dict to a response dict (after performing any
    blocking side-effect such as a motion or a wait)."""
    if "Communication" in req:
        return _handle_communication(req, state, log)
    if "Command" in req:
        return _handle_command(req, state, bridge, log)
    if "Instruction" in req:
        return _handle_instruction(req, state, bridge, log)
    # Unknown shape — be permissive, ack it.
    log(f"  ! unrecognised packet shape: {req!r}")
    return HandlerResult({"ErrorID": 0})


def _handle_communication(req: dict, state: RobotState,
                          log: Callable[[str], None]) -> HandlerResult:
    name = req["Communication"]
    if name == "FRC_Connect":
        # (only expected on the handshake socket, but harmless to answer)
        return HandlerResult({
            "Communication": "FRC_Connect", "ErrorID": 0,
            "PortNumber": DATA_PORT,
            "MajorVersion": RMI_MAJOR_VERSION, "MinorVersion": RMI_MINOR_VERSION,
        })
    if name == "FRC_Disconnect":
        return HandlerResult({"Communication": "FRC_Disconnect", "ErrorID": 0},
                             close_after=True)
    return HandlerResult({"Communication": name, "ErrorID": 0})


def _handle_command(req: dict, state: RobotState, bridge,
                    log: Callable[[str], None]) -> HandlerResult:
    name = req["Command"]
    base = {"Command": name, "ErrorID": 0}

    if name == "FRC_Initialize":
        state.uframe = 0
        state.utool = 0
        return HandlerResult({**base, "GroupMask": 1})

    if name == "FRC_SetOverRide":
        state.override = int(req.get("Value", 100))
        log(f"    override -> {state.override}%")
        try:
            bridge.set_override(state.override)
        except Exception as e:
            log(f"    (bridge.set_override failed: {e})")
        return HandlerResult({"Command": name, "ErrorID": 0})  # u16 ErrorID

    if name == "FRC_SetUFrameUTool":
        state.uframe = int(req.get("UFrameNumber", state.uframe))
        state.utool = int(req.get("UToolNumber", state.utool))
        return HandlerResult({**base, **_group(req)})

    if name == "FRC_GetUFrameUTool":
        return HandlerResult({**base, "UFrameNumber": state.uframe,
                              "UToolNumber": state.utool, **_group(req)})

    if name == "FRC_GetStatus":
        return HandlerResult({
            **base, "ServoReady": 1, "TPMode": 2, "RMIMotionStatus": 0,
            "ProgramStatus": 0, "SingleStepMode": 0,
            "NumberUTool": state.utool, "NumberUFrame": state.uframe,
        })

    if name == "FRC_ReadJointAngles":
        return HandlerResult({**base, "TimeTag": state.time_tag(),
                              "JointAngle": joints_to_wire(state.joints),
                              **_group(req)})

    if name == "FRC_ReadCartesianPosition":
        return HandlerResult({**base, "TimeTag": state.time_tag(),
                              "Configuration": _default_config(state),
                              "Position": _zero_position(), **_group(req)})

    if name == "FRC_ReadTCPSpeed":
        return HandlerResult({**base, "TimeTag": state.time_tag(), "Speed": 0.0})

    if name == "FRC_ReadError":
        return HandlerResult({"Command": name, "ErrorID": 0, "ErrorData": ""})

    if name == "FRC_ReadDIN":
        return HandlerResult({**base, "PortNumber": int(req.get("PortNumber", 0)),
                              "PortValue": 0})

    if name == "FRC_ReadPositionRegister":
        return HandlerResult({**base, "RegisterNumber": int(req.get("RegisterNumber", 0)),
                              "Configuration": _default_config(state),
                              "Position": _zero_position(), **_group(req)})

    if name == "FRC_ReadUFrameData":
        return HandlerResult({**base, "UFrameNumber": int(req.get("FrameNumber", 0)),
                              "Frame": _zero_frame(), **_group(req)})
    if name == "FRC_ReadUToolData":
        return HandlerResult({**base, "UToolNumber": int(req.get("FrameNumber", 0)),
                              "Frame": _zero_frame(), **_group(req)})

    # FRC_Abort, FRC_Reset, FRC_Pause, FRC_Continue, FRC_WriteDOUT,
    # FRC_WriteUFrameData, FRC_WriteUToolData, FRC_WritePositionRegister, ...
    if name in ("FRC_WriteUFrameData", "FRC_WriteUToolData"):
        return HandlerResult({**base, **_group(req)})
    return HandlerResult(base)


def _handle_instruction(req: dict, state: RobotState, bridge,
                        log: Callable[[str], None]) -> HandlerResult:
    name = req["Instruction"]
    seq = _seq_of(req, state)
    resp = {"Instruction": name, "ErrorID": 0, "SequenceID": seq}

    if name == "FRC_SetUTool":
        state.utool = int(req.get("ToolNumber", state.utool))
        log(f"    UTool -> {state.utool}")
        return HandlerResult(resp)
    if name == "FRC_SetUFrame":
        state.uframe = int(req.get("FrameNumber", state.uframe))
        log(f"    UFrame -> {state.uframe}")
        return HandlerResult(resp)
    if name == "FRC_SetPayLoad":
        state.payload = int(req.get("ScheduleNumber", req.get("PayLoad", 0)) or 0)
        return HandlerResult(resp)

    if name == "FRC_WaitTime":
        dt = float(req.get("Time", 0.0))
        log(f"    wait {dt:.3f}s")
        time.sleep(max(dt, 0.0))
        return HandlerResult(resp)

    if name in ("FRC_WaitDIN", "FRC_Call"):
        return HandlerResult(resp)

    # Motion instructions.
    # The *JRep variants carry a "JointAngle" target → we can move directly.
    # The cartesian variants carry "Position"; we have no IK, so we just ack
    # them (the protocol round-trip still works, the arm just doesn't move).
    if "JointAngle" in req and isinstance(req["JointAngle"], dict):
        target = wire_to_joints(req["JointAngle"])
        speed_type = req.get("SpeedType", "")
        speed = float(req.get("Speed", 0.0))
        delta = max(abs(t - c) for t, c in zip(target, state.joints)) if state.joints else 0.0
        dur = _move_duration_s(speed_type, speed, delta, state.override)
        log(f"    move J={_fmt6(target)} via {speed_type}:{speed:g}  (~{dur:.2f}s)")
        _execute_move(state, target, dur, bridge, log)
        return HandlerResult(resp)

    if name.startswith("FRC_") and ("Motion" in name or "Relative" in name):
        log(f"    {name}: cartesian target, no IK in mock — acked, no move")
        return HandlerResult(resp)

    return HandlerResult(resp)


def _execute_move(state: RobotState, target: list[float], dur: float, bridge,
                  log: Callable[[str], None]) -> None:
    """Interpolate joint state current→target over `dur` seconds, pushing each
    intermediate setpoint to the Moveo bridge. The goal pose is also sent up
    front via `bridge.set_goal` — real-arm bridges act on that and run their own
    ramp; the sim bridge ignores it and animates the interpolation instead."""
    start = list(state.joints)
    target = list(target) + [0.0] * (9 - len(target))
    try:
        bridge.set_goal(target, dur)
    except Exception as e:
        log(f"    (bridge.set_goal failed: {e})")
    steps = max(int(dur / 0.02), 1)        # ~50 Hz update
    t0 = time.monotonic()
    for i in range(1, steps + 1):
        f = i / steps
        # smootherstep easing — looks like a real coordinated joint move
        e = f * f * f * (f * (f * 6 - 15) + 10)
        cur = [s + (g - s) * e for s, g in zip(start, target)]
        state.joints = cur
        bridge.set_joints(cur, moving=True)
        # pace to wall clock
        deadline = t0 + dur * f
        sleep = deadline - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
    state.joints = list(target)
    bridge.set_joints(state.joints, moving=False)


# --- small constructors for fields the client model requires -----------------

def _group(req: dict) -> dict:
    g = req.get("Group")
    return {"Group": int(g)} if g is not None else {}


def _default_config(state: RobotState) -> dict:
    return {"UToolNumber": state.utool, "UFrameNumber": state.uframe,
            "Front": 1, "Up": 1, "Left": 1, "Flip": 0,
            "Turn4": 0, "Turn5": 0, "Turn6": 0}


def _zero_position() -> dict:
    return {"X": 0.0, "Y": 0.0, "Z": 0.0, "W": 0.0, "P": 0.0, "R": 0.0,
            "Ext1": 0.0, "Ext2": 0.0, "Ext3": 0.0}


def _zero_frame() -> dict:
    return {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0, "p": 0.0, "r": 0.0}


def _fmt6(j: list[float]) -> str:
    return "[" + ", ".join(f"{v:6.1f}" for v in j[:6]) + "]"


# --- (de)serialisation -------------------------------------------------------

def encode(d: dict[str, Any]) -> bytes:
    return json.dumps(d, separators=(",", ":")).encode("utf-8") + LINE_TERMINATOR


def decode_lines(buf: bytearray) -> list[dict[str, Any]]:
    """Pull complete '<json>\\r\\n' (or '<json>\\n') records out of `buf`,
    mutating it in place. Returns the decoded dicts."""
    out: list[dict[str, Any]] = []
    while True:
        idx = buf.find(b"\n")
        if idx < 0:
            break
        line = bytes(buf[:idx]).strip()
        del buf[:idx + 1]
        if not line:
            continue
        try:
            out.append(json.loads(line.decode("utf-8")))
        except (ValueError, UnicodeDecodeError) as e:
            raise ValueError(f"bad RMI line {line!r}: {e}") from e
    return out
