#!/usr/bin/env python3
"""Read motor speed over the connected CMSIS-DAP probe without stopping the MCU."""

import argparse
import pathlib
import re
import socket
import struct
import subprocess
import sys
import tempfile
import time


ROOT = pathlib.Path(__file__).resolve().parents[1]
ELF = ROOT / "build/stm32_foc_test.elf"
TCL_PORT = 6667


def symbol_address(name):
    symbols = subprocess.check_output(["readelf", "-Ws", str(ELF)], text=True)
    match = re.search(r"(?m)^\s*\d+:\s*([0-9a-fA-F]+)\s+\d+\s+OBJECT\s+GLOBAL\s+\S+\s+\d+\s+" + re.escape(name) + r"$", symbols)
    if match is None:
        raise RuntimeError(f"ELF 中找不到 {name}")
    return int(match.group(1), 16)


def openocd_command():
    return [
        "openocd",
        "-f", "interface/cmsis-dap.cfg",
        "-c", "cmsis_dap_vid_pid 0xc251 0xf001",
        "-f", "target/stm32f1x.cfg",
        "-c", "adapter speed 100",
        "-c", "gdb_port disabled",
        "-c", "telnet_port disabled",
        "-c", f"tcl_port {TCL_PORT}",
        "-c", "init",
    ]


def read_word(connection, address):
    connection.sendall(f"mdw 0x{address:08x} 1\x1a".encode("ascii"))
    response = bytearray()
    while b"\x1a" not in response:
        chunk = connection.recv(4096)
        if not chunk:
            raise RuntimeError("OpenOCD 已斷線")
        response.extend(chunk)
    match = re.search(rb"0x[0-9a-fA-F]+:\s*([0-9a-fA-F]{8})", response)
    if match is None:
        raise RuntimeError(f"SWD 讀取失敗：{response.decode(errors='replace').strip()}")
    return int(match.group(1), 16)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interval", type=float, default=0.2, help="更新間隔，秒（預設 0.2）")
    args = parser.parse_args()
    if args.interval <= 0:
        parser.error("--interval 必須大於 0")

    speed_address = symbol_address("motor_speed")
    fault_address = symbol_address("motor_fault")
    with tempfile.TemporaryFile(mode="w+t") as log:
        process = subprocess.Popen(openocd_command(), stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 5
            while True:
                try:
                    connection = socket.create_connection(("127.0.0.1", TCL_PORT), timeout=1)
                    break
                except OSError:
                    if process.poll() is not None or time.monotonic() >= deadline:
                        log.seek(0)
                        raise RuntimeError("無法連接下載器：\n" + log.read())
                    time.sleep(0.1)

            with connection:
                connection.settimeout(2)
                print("SWD 即時轉速（Ctrl+C 結束）", flush=True)
                while True:
                    start = time.monotonic()
                    speed_bits = read_word(connection, speed_address)
                    fault = read_word(connection, fault_address)
                    speed = struct.unpack("<f", struct.pack("<I", speed_bits))[0]
                    rpm = speed * 60 / (2 * 3.141592653589793)
                    print(f"\rspeed={speed:8.2f} rad/s  rpm={rpm:8.1f}  fault={fault:<2}", end="", flush=True)
                    time.sleep(max(0, args.interval - (time.monotonic() - start)))
        except KeyboardInterrupt:
            print()
        finally:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(error, file=sys.stderr)
        sys.exit(1)
