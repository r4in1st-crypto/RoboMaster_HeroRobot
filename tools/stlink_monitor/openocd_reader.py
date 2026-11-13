"""Read-only application-memory access via a private OpenOCD Tcl RPC process."""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import socket
import struct
import subprocess
import tempfile
import time


def find_openocd(explicit=None):
    if explicit:
        p = Path(explicit).expanduser().resolve()
        if not p.is_file():
            raise FileNotFoundError(f"找不到 OpenOCD：{p}")
        return p
    found = shutil.which("openocd")
    if found:
        return Path(found)
    candidates = []
    for root in (Path("C:/ST"), Path("C:/Program Files/STMicroelectronics")):
        if root.exists():
            candidates.extend(root.glob("STM32CubeIDE*/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.win32*/tools/bin/openocd.exe"))
    if candidates:
        return sorted(candidates)[-1]
    raise FileNotFoundError("未找到 OpenOCD。安装 STM32CubeIDE，或用 --openocd 指定 openocd.exe。")


class OpenOCDReader:
    def __init__(self, executable=None, serial=None):
        self.executable = executable
        self.serial = serial
        self.process = None
        self.sock = None
        self.log_file = None
        self.log_path = None
        self.buffer = b""

    def open(self):
        self.close()
        executable = find_openocd(self.executable)
        with socket.socket() as reserve:
            reserve.bind(("127.0.0.1", 0))
            port = reserve.getsockname()[1]
        args = [str(executable), "-c", f"tcl_port {port}"]
        if self.serial:
            if not all(c.isalnum() or c in "_-" for c in self.serial):
                raise ValueError("ST-Link 序列号只接受字母、数字、下划线及连字符。")
            # Must run after adapter driver selection and before init.
        cfg = Path(__file__).with_name("stlink_readonly.cfg")
        if self.serial:
            text = cfg.read_text(encoding="utf-8").replace("transport select", f"adapter serial {self.serial}\ntransport select")
            handle = tempfile.NamedTemporaryFile(mode="w", suffix=".cfg", encoding="utf-8", delete=False)
            with handle:
                handle.write(text)
            self.temporary_cfg = Path(handle.name)
            cfg = self.temporary_cfg
        log = tempfile.NamedTemporaryFile(prefix="hero_openocd_", suffix=".log", delete=False)
        self.log_path = Path(log.name)
        self.log_file = log
        args.extend(["-f", str(cfg)])
        try:
            self.process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline:
                if self.process.poll() is not None:
                    raise RuntimeError("OpenOCD 未能连接 ST-Link/目标板。" + self.log_tail())
                try:
                    self.sock = socket.create_connection(("127.0.0.1", port), timeout=0.25)
                    self.sock.settimeout(4)
                    break
                except OSError:
                    time.sleep(0.1)
            else:
                raise TimeoutError("等待 OpenOCD 超时。" + self.log_tail())
            self.buffer = b""
            self.command("tcl_notifications off")
            self.command("tcl_trace off")
            return {"openocd": str(executable), "log": str(self.log_path)}
        except Exception:
            self.close()
            raise

    def log_tail(self):
        if self.log_file:
            self.log_file.flush()
        if self.log_path and self.log_path.exists():
            return "\n" + self.log_path.read_text(encoding="utf-8", errors="replace")[-2500:]
        return ""

    def command(self, command):
        if self.sock is None:
            raise ConnectionError("ST-Link 连接未打开。")
        # The caller supplies only fixed commands / checked numeric values.
        request = 'set __hero_rc [catch {' + command + '} __hero_result]; format "%d|%s" $__hero_rc $__hero_result'
        self.sock.sendall(request.encode("ascii") + b"\x1a")
        while b"\x1a" not in self.buffer:
            part = self.sock.recv(65536)
            if not part:
                raise ConnectionError("OpenOCD 连接断开。")
            self.buffer += part
            if len(self.buffer) > 1024 * 1024:
                raise ValueError("OpenOCD 响应长度异常。")
        frame, self.buffer = self.buffer.split(b"\x1a", 1)
        code, sep, result = frame.decode("utf-8", errors="replace").partition("|")
        if sep != "|" or code != "0":
            raise RuntimeError(f"OpenOCD 读取失败：{result or code}")
        return result.strip()

    def state(self):
        self.command("poll")
        return self.command("hero.cpu curstate")

    def read_bytes(self, address, size):
        if not isinstance(address, int) or not isinstance(size, int) or size <= 0:
            raise ValueError("读取地址或长度无效。")
        ranges = ((0x08000000, 0x08200000), (0x20000000, 0x20030000),
                  (0x10000000, 0x10010000), (0xE0042000, 0xE0042004))
        if not any(a <= address and address + size <= b for a, b in ranges):
            raise ValueError(f"读取超出允许区域：0x{address:08X}+{size}")
        start = address & ~3
        end = (address + size + 3) & ~3
        data = bytearray()
        for offset in range(start, end, 1024):
            count = min(1024, end - offset) // 4
            result = self.command(f"read_memory 0x{offset:08x} 32 {count}")
            try:
                words = [int(x, 0) for x in result.split()]
                if len(words) != count or any(not 0 <= x <= 0xffffffff for x in words):
                    raise ValueError()
                data.extend(struct.pack("<" + "I" * count, *words))
            except (ValueError, struct.error) as exc:
                raise ValueError("OpenOCD 返回的内存长度或格式错误。") from exc
        return bytes(data[address - start:address - start + size])

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        # Own process only; do not kill another debugger or send target commands.
        if self.process:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
            self.process = None
        if self.log_file:
            self.log_file.close()
            self.log_file = None
        temp = getattr(self, "temporary_cfg", None)
        if temp:
            temp.unlink(missing_ok=True)
            self.temporary_cfg = None
