"""端到端验证: 仿真 remote_event 语义与固件一致。

用法: conda activate smolvla && python scripts/verify_remote_semantics.py
启动无头仿真(--ik --no-camera) → remote_enable → 逐轴发命令 → get_state/get_ee 断言方向。
"""
import socket
import subprocess
import sys
import time
from pathlib import Path

PORT = 5588
ROOT = Path(__file__).resolve().parent.parent
SIM_CMD = [sys.executable, str(ROOT / "scripts" / "mujoco_sim.py"),
           "--port", str(PORT), "--ik", "--no-camera"]


def send(sock, cmd):
    sock.sendall((cmd + "\n").encode())


def _recv_line(sock):
    buf = b""
    while not buf.endswith(b"\n"):
        buf += sock.recv(1)
    return buf.decode().strip()


def get_state(sock):
    send(sock, "get_state")
    line = _recv_line(sock)
    vals = [float(x) for x in line.split(":", 1)[1].split(",")]
    return vals[:6], vals[6:12]


def get_ee(sock):
    send(sock, "get_ee")
    line = _recv_line(sock)
    return [float(x) for x in line.split(":", 1)[1].split(",")[:3]]


def drive(sock, vals, seconds=0.25, hz=50):
    """按 vals 连续发 remote_event seconds 秒."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        send(sock, "remote_event " + " ".join(f"{v:.3f}" for v in vals))
        time.sleep(1.0 / hz)


def check(name, ok, detail):
    print(f"{'PASS' if ok else 'FAIL'} {name}: {detail}")
    return ok


def main():
    proc = subprocess.Popen(SIM_CMD, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    time.sleep(6)  # 等待模型加载 + TCP 就绪
    sock = socket.create_connection(("127.0.0.1", PORT), timeout=5)
    sock.settimeout(3.0)
    send(sock, "remote_enable")
    time.sleep(1.0)

    results = []

    # 1) vx: p0=-1 → vx=+1 → EE +x
    ee0 = get_ee(sock)
    drive(sock, [-1, 0, 0, 0, 0, 0])
    ee1 = get_ee(sock)
    results.append(check("vx→+x", ee1[0] - ee0[0] > 0.003,
                         f"Δx={(ee1[0]-ee0[0])*1000:.1f}mm"))

    # 2) vy: p1=+1 → vy=+1 → EE +y
    ee0 = get_ee(sock)
    drive(sock, [0, 1, 0, 0, 0, 0])
    ee1 = get_ee(sock)
    results.append(check("vy→+y", ee1[1] - ee0[1] > 0.003,
                         f"Δy={(ee1[1]-ee0[1])*1000:.1f}mm"))

    # 3) vz: p4=1,p5=0 → vz=+0.5 → EE +z
    ee0 = get_ee(sock)
    drive(sock, [0, 0, 0, 0, 1, 0])
    ee1 = get_ee(sock)
    results.append(check("vz→+z", ee1[2] - ee0[2] > 0.001,
                         f"Δz={(ee1[2]-ee0[2])*1000:.1f}mm"))

    # 4) J5: p3=-1 → rx=+1 → J5 角度增大
    a0, _ = get_state(sock)
    drive(sock, [0, 0, 0, -1, 0, 0])
    a1, _ = get_state(sock)
    results.append(check("J5(rx)正转", a1[4] - a0[4] > 1.0,
                         f"ΔJ5={a1[4]-a0[4]:.1f}°"))

    # 5) J6: p2=+1 → ry=+1 → J6 角度增大
    #    (J6 关节阻尼 2.0 > KV=0.3, 速度伺服偏弱, 0.25s 仅积累 ~0.9°; 加长驱动时间)
    a0, _ = get_state(sock)
    drive(sock, [0, 0, 1, 0, 0, 0], seconds=0.45)
    a1, _ = get_state(sock)
    results.append(check("J6(ry)正转", a1[5] - a0[5] > 1.0,
                         f"ΔJ6={a1[5]-a0[5]:.1f}°"))

    sock.close()
    proc.terminate()
    proc.wait(timeout=5)
    print("ALL PASS" if all(results) else f"{sum(results)}/5 PASS")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
