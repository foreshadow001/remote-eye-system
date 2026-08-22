#!/usr/bin/env python3
# ================== recv_data.py (处理主机, Ubuntu) ==================
# 100G 直连接收端: 与 send_data.py (Windows 采集端) 配对。
# 协议:
#   "FILE <relpath> <size> <sha256|0>\n" + <size 字节内容>
#   已存在且大小一致 -> "SKIP" (断点续传); 否则预分配+写入+校验 -> "OK <n>" / "ERR <原因>"
#   "BYE\n" 结束连接
# 用法:
#   python3 recv_data.py --out /data
# 防火墙: sudo ufw allow 5001/tcp (或关闭 ufw)
# =================================================================
import argparse
import hashlib
import os
import socket
import threading
import time
from datetime import datetime
from pathlib import Path

CHUNK = 4 * 1024 * 1024          # 4MB 流式块
SOCK_BUF = 16 * 1024 * 1024      # 接收缓冲

g_lock = threading.Lock()
g_files = g_skips = g_errs = 0
g_bytes = 0
g_t0 = time.time()


def status_line():
    with g_lock:
        el = max(time.time() - g_t0, 1e-6)
        return (f"{g_files} files ({g_bytes/1e12:.2f} TB, {g_bytes/el/1e9:.1f} GB/s) "
                f"skip={g_skips} err={g_errs}")


def log(msg, logf):
    stamp = datetime.now().strftime("%H:%M:%S")
    line = f"[{stamp}] {msg}"
    print(line, flush=True)
    logf.write(line + "\n")
    logf.flush()


def recv_exact(sock, n, f=None, hasher=None):
    """收满 n 字节写入 f; hasher 同步更新; 返回实际收到的字节数"""
    got = 0
    while got < n:
        chunk = sock.recv(min(CHUNK, n - got))
        if not chunk:
            return got
        if f is not None:
            f.write(chunk)
        if hasher is not None:
            hasher.update(chunk)
        got += len(chunk)
    return got


def recv_line(sock):
    buf = b""
    while not buf.endswith(b"\n"):
        d = sock.recv(4096)
        if not d:
            return None
        buf += d
    return buf.decode("utf-8", errors="replace").strip()


def handle_conn(conn, addr, out_root: Path, logf):
    global g_files, g_skips, g_errs, g_bytes
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    conn.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, SOCK_BUF)
    try:
        while True:
            line = recv_line(conn)
            if line is None:
                break
            if line == "BYE":
                conn.sendall(b"BYE\n")
                break
            if not line.startswith("FILE "):
                conn.sendall(f"ERR bad header: {line[:80]}\n".encode())
                continue
            try:
                _, rel, size_s, sha = line.split()
                size = int(size_s)
            except ValueError:
                conn.sendall(b"ERR malformed FILE header\n")
                continue

            dest = (out_root / rel).resolve()
            # 路径安全: 禁止逃逸出 out_root
            if not str(dest).startswith(str(out_root.resolve()) + os.sep):
                conn.sendall(b"ERR path escapes out_root\n")
                continue

            if dest.exists() and dest.stat().st_size == size:
                conn.sendall(b"SKIP\n")
                with g_lock:
                    g_skips += 1
                continue

            dest.parent.mkdir(parents=True, exist_ok=True)
            tmp = dest.with_suffix(dest.suffix + ".part")
            t0 = time.time()
            try:
                hasher = hashlib.sha256() if sha != "0" else None
                with open(tmp, "wb") as f:
                    try:
                        os.posix_fallocate(f.fileno(), 0, size)   # 预分配 (xfs 支持; 失败不致命)
                    except (AttributeError, OSError):             # Windows/不支持时退化为 truncate
                        f.truncate(size)
                    got = recv_exact(conn, size, f, hasher)
                if got != size:
                    tmp.unlink(missing_ok=True)
                    conn.sendall(f"ERR short read {got}/{size}\n".encode())
                    with g_lock:
                        g_errs += 1
                    continue
                if hasher is not None and hasher.hexdigest() != sha:
                    tmp.unlink(missing_ok=True)
                    conn.sendall(b"ERR sha256 mismatch\n")
                    with g_lock:
                        g_errs += 1
                    log(f"ERR  {rel}: sha256 mismatch", logf)
                    continue
                os.replace(tmp, dest)          # 原子落位
                conn.sendall(f"OK {size}\n".encode())
                with g_lock:
                    g_files += 1
                    g_bytes += size
                log(f"OK   {rel} ({size/1e9:.2f} GB, {size/1e6/max(time.time()-t0,1e-6):.0f} MB/s)", logf)
            except OSError as e:
                tmp.unlink(missing_ok=True)
                conn.sendall(f"ERR {e}\n".encode())
                with g_lock:
                    g_errs += 1
                log(f"ERR  {rel}: {e}", logf)
    except ConnectionError:
        pass
    finally:
        conn.close()


def main():
    ap = argparse.ArgumentParser(description="Receive h5 capture data from acquisition hosts")
    ap.add_argument("--out", default="/data", help="output root (default /data)")
    ap.add_argument("--port", type=int, default=5001)
    ap.add_argument("--bind", default="0.0.0.0")
    args = ap.parse_args()

    out_root = Path(args.out)
    out_root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    logf = open(out_root / f"transfer_{stamp}.log", "a", encoding="utf-8")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.bind, args.port))
    srv.listen(8)
    log(f"recv_data listening on {args.bind}:{args.port} -> {out_root}", logf)
    print("Ctrl+C 停止 (断开的连接会由 sender 重试, 已收文件 SKIP 续传)", flush=True)

    try:
        while True:
            conn, addr = srv.accept()
            log(f"conn from {addr[0]}:{addr[1]}", logf)
            threading.Thread(target=handle_conn, args=(conn, addr, out_root, logf), daemon=True).start()
            print(f"[{datetime.now().strftime('%H:%M:%S')}] {status_line()}", flush=True)
    except KeyboardInterrupt:
        print(f"\n=== stopped: {status_line()} ===")
        logf.close()


if __name__ == "__main__":
    main()
