#!/usr/bin/env python3
"""
plot_cart_goal.py ────────────────────────────────────────────
ROS 2 bag 에 기록된 '/cart_goal' 토픽만 읽어,
 - 0~2 (x,y,z) → 첫 번째 서브플롯
 - 3~5 (r,p,y) → 두 번째 서브플롯 (rad→deg 변환)
으로 플롯합니다.
─────────────────────────────────────────────────────────────
"""
import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# 기본 BAG 경로 (원하는 경로로 수정하거나 실행 시 인자로 넘길 수 있음)
DEFAULT_BAG_DIR = '/home/airplon/franka_ws/franka_example_controllers/bagfiles/force_pd'

def load_cart_goal(bag_dir):
    bag_dir = Path(bag_dir).expanduser().resolve()
    if not bag_dir.is_dir():
        raise FileNotFoundError(f"{bag_dir} is not a directory")

    # rosbag2 reader 설정
    storage_opts   = rosbag2_py._storage.StorageOptions(uri=str(bag_dir), storage_id='sqlite3')
    converter_opts = rosbag2_py._storage.ConverterOptions('', 'cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_opts, converter_opts)

    # topic → 타입 매핑
    type_map = {info.name: info.type for info in reader.get_all_topics_and_types()}

    times = []
    values = []
    topic = '/cart_goal'
    if topic not in type_map:
        raise RuntimeError(f"Topic {topic} not found in bag")

    # 메시지 역직렬화
    while reader.has_next():
        name, raw, ts = reader.read_next()
        if name != topic:
            continue
        msg_cls = get_message(type_map[topic])
        msg     = deserialize_message(raw, msg_cls)
        times.append(ts * 1e-9)      # ns → s
        values.append(list(msg.data))  # 길이 6 리스트

    return times, values

def plot_cart_goal(times, vals):
    # 2×1 subplot
    fig, (ax_xyz, ax_rpy) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    # 0~2 : x,y,z
    labels_xyz = ['x_goal', 'y_goal', 'z_goal']
    for i, lbl in enumerate(labels_xyz):
        ax_xyz.plot(times, [v[i] for v in vals], label=lbl)
    ax_xyz.set_title('cart_goal position (0~2)')
    ax_xyz.set_ylabel('meter')
    ax_xyz.grid(True, linestyle='--', alpha=0.4)
    ax_xyz.legend()

    # 3~5 : yaw,pitch,roll → deg 변환
    labels_rpy = ['yaw_goal°', 'pitch_goal°', 'roll_goal°']
    for offset, lbl in zip(range(3, 6), labels_rpy):
        ax_rpy.plot(times,
                    [math.degrees(v[offset]) for v in vals],
                    label=lbl)
    ax_rpy.set_title('cart_goal orientation (3~5) in degrees')
    ax_rpy.set_xlabel('time [s]')
    ax_rpy.set_ylabel('degree')
    ax_rpy.grid(True, linestyle='--', alpha=0.4)
    ax_rpy.legend()

    plt.tight_layout()
    plt.show()

def main():
    parser = argparse.ArgumentParser(
        description="Plot only '/cart_goal' from ROS2 bag")
    parser.add_argument(
        'bag_dir', nargs='?', default=DEFAULT_BAG_DIR,
        help=f"bag folder (default: {DEFAULT_BAG_DIR})")
    args = parser.parse_args()

    print(f"▶ reading '/cart_goal' from {args.bag_dir}")
    times, vals = load_cart_goal(args.bag_dir)
    if not times:
        print("No '/cart_goal' messages found.")
        return

    print("▶ plotting '/cart_goal' …")
    plot_cart_goal(times, vals)

if __name__ == '__main__':
    main()
