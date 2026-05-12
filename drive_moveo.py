#!/usr/bin/env python3
"""
Drive the (mock) FANUC controller with the real `valstad-shipworks/fanuc_ucl`
library — the same client you'd point at an actual R-30iB.

    pip install fanuc_ucl
    python run_mock.py                       # terminal 1
    python drive_moveo.py --ip 127.0.0.1     # terminal 2

This is intentionally close to fanuc_ucl's own examples/rmi.py: connect → reset
→ initialize → set override / tool / frame → a short joint-motion program →
read back joint angles. The mock executes the moves and forwards them to the
Moveo bridge, so a physical (or simulated) arm follows along.

Note: the fanuc_ucl RMI driver always connects to <ip>:16001 for the handshake;
the data port is whatever the controller reports (the mock reports 16002).
"""

import argparse
import sys


def jrep(rmi, JointFormat, JointTemplate, j6, speed_ms=600):
    """Build an FRC_JointMotionJRep instruction for a 6-axis target (deg, abs)."""
    return rmi.FrcJointMotionJRep(
        rmi.JointAngles(JointFormat.AbsDeg, JointTemplate.SIX, *j6),
        rmi.SpeedType.MilliSeconds, speed_ms, rmi.TermType.FINE, 0,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="fanuc_ucl RMI demo against the mock")
    ap.add_argument("--ip", default="127.0.0.1",
                    help="controller IP (handshake port 16001 is fixed by fanuc_ucl)")
    ap.add_argument("--speed-ms", type=int, default=700,
                    help="per-move duration in ms (SpeedType.MilliSeconds)")
    args = ap.parse_args()
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass

    try:
        from fanuc_ucl import JointFormat, JointTemplate, ThreadConfig, rmi
    except ImportError:
        print("fanuc_ucl is not installed.  ->  pip install fanuc_ucl", file=sys.stderr)
        return 1

    print(f"connecting to FANUC RMI at {args.ip}:16001 …")
    driver = rmi.RmiDriver(rmi.RmiDriverConfig(args.ip))
    info = driver.connect(ThreadConfig(80, None))
    print(f"connected — controller RMI v{info.major_version}.{info.minor_version}, "
          f"data port {info.port_number}")

    driver.send_full_reset().wait_timeout(20.0)
    driver.send(rmi.FrcInitialize()).wait_timeout(20.0)
    driver.send(rmi.FrcSetOverRide(60)).wait_timeout(2.0)
    driver.send(rmi.FrcSetUTool(1)).wait_timeout(2.0)
    driver.send(rmi.FrcSetUFrame(1)).wait_timeout(2.0)
    print("initialised. running motion program …")

    # A little "look around / reach / return" sequence. AbsDeg, J1..J6.
    program = [
        [   0.0,  20.0,  10.0,    0.0,  30.0,    0.0],
        [  60.0,  35.0, -10.0,   45.0,  20.0,   90.0],
        [ -60.0,  35.0, -10.0,  -45.0,  20.0,  -90.0],
        [  30.0,  60.0, -25.0,   20.0,  60.0,   45.0],
        [   0.0,   0.0,   0.0,    0.0,   0.0,    0.0],
    ]
    sec = args.speed_ms / 1000.0
    for i, tgt in enumerate(program, 1):
        print(f"  move {i}/{len(program)} → {tgt}")
        driver.send(jrep(rmi, JointFormat, JointTemplate, tgt, args.speed_ms)) \
              .wait_timeout(sec + 1.0)
        driver.send(rmi.FrcWaitTime(0.3)).wait()

    resp = driver.send(rmi.FrcReadJointAngles()).wait_timeout(1.0)
    if resp is not None:
        ja = resp.joints(JointFormat.AbsDeg, JointTemplate.SIX).as_array()
        print(f"final joint angles (AbsDeg): "
              f"[{', '.join(f'{v:.1f}' for v in ja[:6])}]")

    # Clean shutdown — sends FRC_Disconnect and joins the driver's runner thread.
    try:
        driver.disconnect()
    except Exception:
        pass
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
