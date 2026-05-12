"""
Mock FANUC R-30iB controller — TCP server.

Listens on two ports (mirroring a real RMI-enabled controller):
  * 16001  handshake: accept one FRC_Connect, reply with the data port, close.
  * 16002  data: a stream of Command / Instruction / Communication packets;
           we answer each, strictly FIFO. Motion / wait instructions block
           until done before we reply (matches fanuc_ucl's blocking RMI mode).

Single session at a time on the data port — fine for a demo. Robot state and
the Moveo bridge are shared via a lock.
"""

from __future__ import annotations

import socket
import threading
import time
from typing import Optional

from .protocol import (HANDSHAKE_PORT, DATA_PORT, RMI_MAJOR_VERSION,
                       RMI_MINOR_VERSION, RobotState, handle_packet,
                       encode, decode_lines)
from .moveo_bridge import MoveoBridge


def _ts() -> str:
    return time.strftime("%H:%M:%S")


class MockFanucController:
    def __init__(self, bridge: MoveoBridge, host: str = "0.0.0.0",
                 handshake_port: int = HANDSHAKE_PORT,
                 data_port: int = DATA_PORT, quiet: bool = False):
        self.bridge = bridge
        self.host = host
        self.handshake_port = handshake_port
        self.data_port = data_port
        self.quiet = quiet
        self.state = RobotState()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []

    # -- logging --------------------------------------------------------------
    def log(self, msg: str) -> None:
        if not self.quiet:
            print(f"[{_ts()}] {msg}", flush=True)

    # -- lifecycle ------------------------------------------------------------
    def serve_forever(self) -> None:
        hs = self._make_listener(self.handshake_port)
        dt = self._make_listener(self.data_port)
        self.log(f"mock FANUC controller up — handshake :{self.handshake_port}"
                 f"  data :{self.data_port}  (RMI v{RMI_MAJOR_VERSION}."
                 f"{RMI_MINOR_VERSION})")
        self.log("waiting for fanuc_ucl client …")
        t1 = threading.Thread(target=self._handshake_loop, args=(hs,), daemon=True)
        t2 = threading.Thread(target=self._data_loop, args=(dt,), daemon=True)
        t1.start(); t2.start()
        self._threads = [t1, t2]
        try:
            while not self._stop.is_set():
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        finally:
            self._stop.set()
            for s in (hs, dt):
                try:
                    s.close()
                except OSError:
                    pass
            try:
                self.bridge.close()
            except Exception:
                pass
            self.log("controller stopped.")

    def _make_listener(self, port: int) -> socket.socket:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, port))
        s.listen(4)
        s.settimeout(0.5)
        return s

    # -- handshake port -------------------------------------------------------
    def _handshake_loop(self, lst: socket.socket) -> None:
        while not self._stop.is_set():
            try:
                conn, addr = lst.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._serve_handshake, args=(conn, addr),
                             daemon=True).start()

    def _serve_handshake(self, conn: socket.socket, addr) -> None:
        conn.settimeout(2.0)
        buf = bytearray()
        try:
            while not self._stop.is_set():
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    break
                if not chunk:
                    break
                buf.extend(chunk)
                pkts = decode_lines(buf)
                for req in pkts:
                    if req.get("Communication") == "FRC_Connect":
                        self.log(f"handshake from {addr[0]} → port {self.data_port}, "
                                 f"RMI v{RMI_MAJOR_VERSION}.{RMI_MINOR_VERSION}")
                        conn.sendall(encode({
                            "Communication": "FRC_Connect", "ErrorID": 0,
                            "PortNumber": self.data_port,
                            "MajorVersion": RMI_MAJOR_VERSION,
                            "MinorVersion": RMI_MINOR_VERSION}))
                    else:
                        conn.sendall(encode({"ErrorID": 0}))
                if pkts:
                    break
        except OSError:
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    # -- data port ------------------------------------------------------------
    def _data_loop(self, lst: socket.socket) -> None:
        while not self._stop.is_set():
            try:
                conn, addr = lst.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.log(f"data session opened from {addr[0]}:{addr[1]}")
            try:
                self._serve_data(conn, addr)
            finally:
                self.log("data session closed.")

    def _serve_data(self, conn: socket.socket, addr) -> None:
        conn.settimeout(0.5)
        buf = bytearray()
        try:
            while not self._stop.is_set():
                try:
                    chunk = conn.recv(8192)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buf.extend(chunk)
                try:
                    reqs = decode_lines(buf)
                except ValueError as e:
                    self.log(f"  ! {e}")
                    continue
                for req in reqs:
                    name = (req.get("Instruction") or req.get("Command")
                            or req.get("Communication") or "?")
                    seq = req.get("SequenceID")
                    self.log(f"  → {name}" + (f"  seq={seq}" if seq is not None else "")
                             + f"  {self._brief(req)}")
                    with self._lock:
                        result = handle_packet(req, self.state, self.bridge, self.log)
                    conn.sendall(encode(result.response))
                    self.log(f"  ← {self._brief(result.response)}")
                    if result.close_after:
                        return
        except OSError:
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    @staticmethod
    def _brief(d: dict) -> str:
        skip = {"Communication", "Command", "Instruction", "SequenceID"}
        items = {k: v for k, v in d.items() if k not in skip}
        if "JointAngle" in items and isinstance(items["JointAngle"], dict):
            ja = items["JointAngle"]
            items["JointAngle"] = "[" + ",".join(f"{ja.get(k,0):g}" for k in
                                                  ("J1", "J2", "J3", "J4", "J5", "J6")) + "]"
        s = ", ".join(f"{k}={v}" for k, v in items.items())
        return s if len(s) <= 90 else s[:87] + "…"
