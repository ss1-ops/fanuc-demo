#!/usr/bin/env python3
"""
Start the mock FANUC R-30iB controller.

    python run_mock.py                              # sim — no hardware
    python run_mock.py --moveo pi  --pi-host armpi.local      # → Pi's moveo_publisher.py:9000
    python run_mock.py --moveo ros2                # → /joint_commands directly (needs rclpy)
    python run_mock.py --host 10.0.0.1             # bind a specific address

Then point a fanuc_ucl client at it (the library always connects to <ip>:16001
for the handshake):
    python drive_moveo.py --ip 127.0.0.1
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from mock_fanuc.server import MockFanucController          # noqa: E402
from mock_fanuc.moveo_bridge import make_bridge            # noqa: E402
from mock_fanuc.protocol import HANDSHAKE_PORT, DATA_PORT  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="Mock FANUC RMI controller → BCN3D Moveo")
    ap.add_argument("--host", default="0.0.0.0",
                    help="bind address (default 0.0.0.0; use 10.0.0.1 to match "
                         "fanuc_ucl's example after aliasing it to loopback)")
    ap.add_argument("--handshake-port", type=int, default=HANDSHAKE_PORT)
    ap.add_argument("--data-port", type=int, default=DATA_PORT)
    ap.add_argument("--moveo", choices=["sim", "ros2", "pi"], default="sim",
                    help="where joint setpoints go: sim (no hw, logs+runtime/), "
                         "ros2 (direct rclpy publish to /joint_commands + "
                         "/speed_scale), pi (TCP to moveo_publisher.py on the Pi)")
    ap.add_argument("--pi-host", default=os.environ.get("MOVEO_PI_HOST"),
                    help="Pi hostname/IP for --moveo pi (or set $MOVEO_PI_HOST)")
    ap.add_argument("--pi-port", type=int, default=9000,
                    help="moveo_publisher.py TCP port (default 9000)")
    ap.add_argument("--ros2-joint-topic", default="/joint_commands")
    ap.add_argument("--ros2-speed-topic", default="/speed_scale")
    ap.add_argument("--runtime-dir", default=os.path.join(HERE, "runtime"))
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    bridge = make_bridge(args.moveo, runtime_dir=args.runtime_dir,
                         verbose=not args.quiet,
                         ros2_joint_topic=args.ros2_joint_topic,
                         ros2_speed_topic=args.ros2_speed_topic,
                         pi_host=args.pi_host, pi_port=args.pi_port)
    ctl = MockFanucController(bridge, host=args.host,
                              handshake_port=args.handshake_port,
                              data_port=args.data_port, quiet=args.quiet)
    ctl.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
