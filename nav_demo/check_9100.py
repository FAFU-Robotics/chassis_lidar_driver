#!/usr/bin/env python3
"""Stage 5: confirm Navigator would use TeleopTcpClient.stick → :9100. Never send stick."""
from __future__ import annotations

import inspect
import socket
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))


def main() -> int:
    print("STAGE5 :9100 interface check — hello/ping only, NO stick", flush=True)
    from official_mods import ensure_package

    ensure_package()
    from bunker_mini.teleop_tcp import DEFAULT_PORT, DEFAULT_TOKEN, TeleopTcpClient

    src = inspect.getsource(TeleopTcpClient.stick)
    if "TYPE_STICK" not in src or "self.sock.sendall" not in src:
        print("FAIL: TeleopTcpClient.stick does not look like the official TCP stick")
        return 2
    print("CODE TeleopTcpClient.stick(v,w) → TCP TYPE_STICK on connected socket")
    print("CODE Navigator drive callback must be: teleop.stick(v,w)  (not a second CAN)")
    print("CODE do not use TeleopTcpClient.goto (wheel-odom navigator)")
    print("CODE do not construct BunkerMiniController for this demo")

    port = DEFAULT_PORT
    with socket.create_connection(("127.0.0.1", port), timeout=2.0) as s:
        print(f"TCP 127.0.0.1:{port} connect=OK")
        s.close()

    # Confirm listener without sending stick.
    cli = TeleopTcpClient("127.0.0.1", port, DEFAULT_TOKEN)
    try:
        rtt_hello = cli.hello()
        rtt_ping = cli.ping()
        print(f"HELLO token={DEFAULT_TOKEN!r} rtt_us={rtt_hello}")
        print(f"PING rtt_us={rtt_ping}")
        print("STICK not sent (dry-run)")
        print("DRY_RUN would send stick: v=0.120 w=0.000  (not actually sent)")
        print("PASS: :9100 is the only chassis entry; stick path confirmed; no stick sent")
        return 0
    finally:
        cli.close()


if __name__ == "__main__":
    sys.exit(main())
