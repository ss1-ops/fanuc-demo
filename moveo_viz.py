#!/usr/bin/env python3
"""
Live 3D stick-figure of the Moveo arm following the mock FANUC controller.

Reads runtime/moveo_state.json (written by the SimMoveoBridge) and animates an
approximate forward-kinematics rendering of the 6-DOF arm. Link lengths are
rough Moveo dimensions — this is for "watch it move", not for accuracy.

    python run_mock.py            # terminal 1
    python moveo_viz.py           # terminal 2  (this)
    python drive_moveo.py         # terminal 3  -> the arm moves

If you have the real arm + ROS 2, use RViz with your URDF instead and run the
mock with `--moveo ros2`.
"""

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

# Moveo link lengths (mm), from the arm code's IK chain (_L_BASE/_L_UPPER/_L_FORE/_L_EE):
# base column → shoulder pivot, upper arm, forearm, wrist-pitch pivot → tool tip.
L0Z, L2, L3, L4 = 230.0, 228.0, 235.0, 40.0


def _rz(theta):
    c, s = math.cos(theta), math.sin(theta)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _mv(m, v):
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


def fk(moveo_deg):
    """[waist, shoulder, elbow, wrist_roll, wrist_pitch] (deg) ->
    list of 3D points base→shoulder→elbow→wrist→tip (mm). wrist_roll doesn't
    change the stick shape, so only indices 0,1,2,4 are used."""
    yaw = math.radians(moveo_deg[0])
    a2 = math.radians(moveo_deg[1])
    a3 = math.radians(moveo_deg[2])
    a5 = math.radians(moveo_deg[4])
    R = _rz(yaw)
    p_base = (0.0, 0.0, 0.0)
    p_sh = (0.0, 0.0, L0Z)
    d2 = _mv(R, (math.cos(a2), 0.0, math.sin(a2)))
    p_el = tuple(p_sh[i] + L2 * d2[i] for i in range(3))
    d3 = _mv(R, (math.cos(a2 + a3), 0.0, math.sin(a2 + a3)))
    p_wr = tuple(p_el[i] + L3 * d3[i] for i in range(3))
    d5 = _mv(R, (math.cos(a2 + a3 + a5), 0.0, math.sin(a2 + a3 + a5)))
    p_tp = tuple(p_wr[i] + L4 * d5[i] for i in range(3))
    return [p_base, p_sh, p_el, p_wr, p_tp]


def main() -> int:
    ap = argparse.ArgumentParser(description="Live Moveo stick-figure viewer")
    ap.add_argument("--state", default=os.path.join(HERE, "runtime", "moveo_state.json"))
    ap.add_argument("--interval-ms", type=int, default=40)
    args = ap.parse_args()

    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        print("matplotlib is required for the viewer  ->  pip install matplotlib",
              file=sys.stderr)
        return 1

    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="3d")
    reach = L2 + L3 + L4 + 50
    (line,) = ax.plot([], [], [], "-o", lw=4, ms=8, color="#e94560")
    title = ax.set_title("Moveo — waiting for run_mock.py / drive_moveo.py …")

    def setup_axes():
        ax.set_xlim(-reach, reach)
        ax.set_ylim(-reach, reach)
        ax.set_zlim(0, reach + L0Z * 0.5)
        ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)"); ax.set_zlabel("z (mm)")
        ax.set_box_aspect((1, 1, 0.9))

    setup_axes()

    def update(_frame):
        try:
            with open(args.state) as fh:
                st = json.load(fh)
        except (OSError, ValueError):
            return line, title
        pts = fk(st.get("moveo_deg", [0] * 6))
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
        line.set_data(xs, ys); line.set_3d_properties(zs)
        moving = st.get("moving")
        names = st.get("moveo_joint_names", [])
        deg = st.get("moveo_deg", [])
        txt = "  ".join(f"{n}={v:.0f}°" for n, v in zip(names, deg))
        title.set_text(("● moving  " if moving else "○ idle    ") + txt)
        line.set_color("#1aa3ff" if moving else "#e94560")
        return line, title

    _anim = FuncAnimation(fig, update, interval=args.interval_ms, blit=False,
                          cache_frame_data=False)
    plt.tight_layout()
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
