#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_attitude.py - STM32F103C8T6 + MPU6050 上位机工具

功能：
    1) 从串口读取下位机输出的 $ATT 数据帧；
    2) 实时绘制 Roll / Pitch / Yaw 姿态曲线；
    3) 可选把全部数据保存为 CSV 文件（可直接用 Excel 打开）。

数据帧格式（固件中由 APP_PrintAttitude() 输出）：
    $ATT,t_ms,roll,pitch,yaw,gx,gy,gz,ax,ay,az,temp

依赖：
    pip install pyserial matplotlib

用法示例：
    python plot_attitude.py --port COM3                     # 实时绘图
    python plot_attitude.py --port COM3 --save data.csv     # 同时保存 CSV
    python plot_attitude.py --port COM3 --no-plot --save d.csv
    python plot_attitude.py --list                          # 列出可用串口
"""

from __future__ import annotations

import argparse
import sys
from collections import deque

# ---------------------------------------------------------------------------
# 常量定义
# ---------------------------------------------------------------------------
FRAME_PREFIX = "$ATT,"
FIELD_NAMES = [
    "t_ms", "roll", "pitch", "yaw",
    "gx", "gy", "gz",
    "ax", "ay", "az",
    "temp",
]
ANGLE_FIELDS = ("roll", "pitch", "yaw")
WINDOW_SECONDS = 20.0  # 绘图窗口长度（秒）


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="读取 STM32 串口姿态数据并实时绘图",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("-p", "--port", help="串口号，例如 COM3 或 /dev/ttyUSB0")
    parser.add_argument("-b", "--baudrate", type=int, default=115200,
                        help="波特率，需与固件一致")
    parser.add_argument("--save", metavar="FILE", default=None,
                        help="把接收到的数据保存为 CSV 文件")
    parser.add_argument("--no-plot", action="store_true",
                        help="只接收数据不绘图（适合后台记录）")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="采集时长（秒），0 表示一直采集直到 Ctrl+C")
    parser.add_argument("--list", action="store_true",
                        help="列出当前可用的串口后退出")
    return parser.parse_args()


def list_ports() -> int:
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        print("未安装 pyserial，请先执行：pip install pyserial", file=sys.stderr)
        return 1

    ports = list(lp.comports())
    if not ports:
        print("未发现可用串口。")
        return 0

    print("可用串口：")
    for p in ports:
        print(f"  {p.device:<12} {p.description}")
    return 0


def parse_frame(line: str):
    """把一行文本解析成数据字典，解析失败返回 None。"""
    if not line.startswith(FRAME_PREFIX):
        return None

    parts = line[len(FRAME_PREFIX):].split(",")
    if len(parts) != len(FIELD_NAMES):
        return None

    try:
        values = [float(v) for v in parts]
    except ValueError:
        return None

    return dict(zip(FIELD_NAMES, values))


def main() -> int:
    args = parse_args()

    if args.list:
        return list_ports()

    if not args.port:
        print("错误：请用 --port 指定串口，或用 --list 查看可用串口。",
              file=sys.stderr)
        return 2

    try:
        import serial
    except ImportError:
        print("未安装 pyserial，请先执行：pip install pyserial", file=sys.stderr)
        return 1

    try:
        port = serial.Serial(args.port, args.baudrate, timeout=1.0)
    except Exception as exc:  # noqa: BLE001
        print(f"打开串口 {args.port} 失败：{exc}", file=sys.stderr)
        return 1

    print(f"已连接 {args.port} @ {args.baudrate}，等待数据帧 {FRAME_PREFIX}* ...")
    print("按 Ctrl+C 结束。")

    csv_file = None
    csv_writer = None
    if args.save:
        csv_file = open(args.save, "w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(FIELD_NAMES)

    # ---------------- 无绘图模式：纯记录 ----------------
    if args.no_plot:
        return run_record_only(port, csv_writer, csv_file, args.duration)

    # ---------------- 绘图模式 ----------------
    try:
        import matplotlib
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError:
        print("未安装 matplotlib，请先执行：pip install matplotlib\n"
              "或使用 --no-plot 仅保存 CSV。", file=sys.stderr)
        if csv_file:
            csv_file.close()
        return 1

    # 中文显示：优先使用微软雅黑 / 文泉驿，缺失时自动回退到英文
    matplotlib.rcParams["font.sans-serif"] = [
        "Microsoft YaHei", "SimHei", "WenQuanYi Micro Hei", "DejaVu Sans",
    ]
    matplotlib.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    fig.canvas.manager.set_window_title("MPU6050 Attitude (STM32F103C8T6)")
    fig.suptitle("MPU6050 Attitude - Roll / Pitch / Yaw", fontsize=12)

    colors = {"roll": "#d62728", "pitch": "#2ca02c", "yaw": "#1f77b4"}
    lines = {}
    for ax, field in zip(axes, ANGLE_FIELDS):
        (line,) = ax.plot([], [], color=colors[field], linewidth=1.2)
        lines[field] = line
        ax.set_ylabel(f"{field} (°)")
        ax.grid(True, linestyle=":", alpha=0.6)
        ax.set_ylim(-180, 180)

    axes[-1].set_xlabel("时间 (s)")

    buf_t = deque(maxlen=20000)
    buf = {f: deque(maxlen=20000) for f in ANGLE_FIELDS}
    counter = {"frames": 0, "start": None, "last_log": 0.0}

    def read_and_store():
        """读取串口缓冲区中所有完整的行并入库。"""
        try:
            raw = port.readline()
        except Exception as exc:  # noqa: BLE001
            print(f"串口读取异常：{exc}", file=sys.stderr)
            return

        if not raw:
            return

        text = raw.decode("ascii", errors="ignore").strip()
        if not text:
            return

        data = parse_frame(text)
        if data is None:
            return

        if counter["start"] is None:
            counter["start"] = data["t_ms"]

        t = (data["t_ms"] - counter["start"]) / 1000.0
        buf_t.append(t)
        for f in ANGLE_FIELDS:
            buf[f].append(data[f])
        counter["frames"] += 1

        if csv_writer is not None:
            csv_writer.writerow([data[f] for f in FIELD_NAMES])

    def update(_frame):
        # 一次动画回调里尽量把缓冲区读空，保证图形跟得上数据流
        for _ in range(64):
            if port.in_waiting == 0:
                break
            read_and_store()

        if not buf_t:
            return tuple(lines.values())

        t_now = buf_t[-1]
        t_min = max(0.0, t_now - WINDOW_SECONDS)

        # 找到窗口起点，只保留可见范围内的数据
        start_idx = 0
        for i, tv in enumerate(buf_t):
            if tv >= t_min:
                start_idx = i
                break

        xs = list(buf_t)[start_idx:]
        for f in ANGLE_FIELDS:
            lines[f].set_data(xs, list(buf[f])[start_idx:])

        for ax in axes:
            ax.set_xlim(t_min, max(t_min + 1.0, t_now))

        if counter["frames"] and (t_now - counter["last_log"] >= 2.0):
            counter["last_log"] = t_now
            print(f"已接收 {counter['frames']} 帧，当前 "
                  f"roll={buf['roll'][-1]:.2f}  "
                  f"pitch={buf['pitch'][-1]:.2f}  "
                  f"yaw={buf['yaw'][-1]:.2f}")

        return tuple(lines.values())

    ani = FuncAnimation(fig, update, interval=50, blit=False,
                        cache_frame_data=False)

    if args.duration > 0:
        timer = fig.canvas.new_timer(interval=int(args.duration * 1000))
        timer.add_callback(lambda: plt.close(fig))
        timer.start()

    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        port.close()
        if csv_file:
            csv_file.close()
            print(f"\n数据已保存到 {args.save}")

    _ = ani  # 保持引用，防止被垃圾回收
    return 0


def run_record_only(port, csv_writer, csv_file, duration: float) -> int:
    """只记录数据，不绘图。"""
    import time

    frames = 0
    start = time.time()
    try:
        while True:
            raw = port.readline()
            if raw:
                text = raw.decode("ascii", errors="ignore").strip()
                data = parse_frame(text)
                if data is not None:
                    frames += 1
                    if csv_writer is not None:
                        csv_writer.writerow([data[f] for f in FIELD_NAMES])

            if duration > 0 and (time.time() - start) >= duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        port.close()
        if csv_file:
            csv_file.close()

    print(f"采集结束，共 {frames} 帧" + (f"，已保存到 {csv_file.name}" if csv_file else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
