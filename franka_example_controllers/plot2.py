#!/usr/bin/env python3
"""
plot_franka_bag_goal.py
──────────────────────────────────────────────────────────────
* /ee_position          : x,y,z  +  cart_goal[0:3] (black dashed)
* /ee_orientation_rpy   : yaw,pitch,roll(°)  +  cart_goal[3:6] (black dashed, °)
* /cart_pos_err         : 6-dim error
* /tau_total            : 7-dim torque
──────────────────────────────────────────────────────────────
"""
import argparse, math
from pathlib import Path
import matplotlib.pyplot as plt

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# 기본 BAG 경로
DEFAULT_BAG_DIR = '/home/airplon/franka_ws/franka_example_controllers/bagfiles/force_pd'

# ───── 토픽 목록 ─────
TOPICS = [
    '/ee_position',
    '/ee_orientation_rpy',
    '/cart_pos_err',
    '/tau_total',
    '/cart_goal',                    # ← 신규
]

# 시각화용 라벨
LABELS = {
    '/ee_position':        ['x', 'y', 'z'],
    '/ee_orientation_rpy': ['yaw°', 'pitch°', 'roll°'],
    '/cart_pos_err':       [f'err{i}' for i in range(6)],
    '/tau_total':          [f'τ{i}'   for i in range(7)],
}

# ───── BAG 로드 ─────
def load_topics(bag_dir):
    bag_dir = Path(bag_dir).expanduser().resolve()
    if not bag_dir.is_dir():
        raise FileNotFoundError(bag_dir)

    storage_opts   = rosbag2_py._storage.StorageOptions(uri=str(bag_dir),
                                                        storage_id='sqlite3')
    converter_opts = rosbag2_py._storage.ConverterOptions('', 'cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_opts, converter_opts)

    type_map = {info.name: info.type for info in reader.get_all_topics_and_types()}
    t_buf, d_buf = {t: [] for t in TOPICS}, {t: [] for t in TOPICS}

    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        if topic not in TOPICS:
            continue

        msg_cls = get_message(type_map[topic])
        msg     = deserialize_message(raw, msg_cls)
        t_buf[topic].append(stamp_ns * 1e-9)          # ns → s

        if topic == '/ee_position':
            d_buf[topic].append((msg.point.x, msg.point.y, msg.point.z))

        elif topic == '/ee_orientation_rpy':
            d_buf[topic].append([math.degrees(v) for v in msg.data])

        elif topic == '/cart_goal':
            vals = list(msg.data)                     # 0-5
            # 3-5(rad) → deg
            for i in (3, 4, 5):
                vals[i] = math.degrees(vals[i])
            d_buf[topic].append(vals)

        else:  # 나머지 Float64MultiArray
            d_buf[topic].append(list(msg.data))

    return t_buf, d_buf


# ───── 플롯 ─────
def make_plot(t_buf, d_buf):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex='col')
    ax_pos, ax_rpy, ax_err, ax_tau = axes.flatten()

    # 1) EE 위치
    t = t_buf['/ee_position']
    v = d_buf['/ee_position']
    for i, lbl in enumerate(LABELS['/ee_position']):
        ax_pos.plot(t, [vv[i] for vv in v], label=lbl)
    # cart_goal 0-2
    tg = t_buf['/cart_goal']; vg = d_buf['/cart_goal']
    if tg:
        for i, lbl in enumerate(['x_goal', 'y_goal', 'z_goal']):
            ax_pos.plot(tg, [g[i] for g in vg],
                        '--', color='k', linewidth=1, label=lbl)
    ax_pos.set_title('ee_position')
    ax_pos.grid(True, linestyle='--', alpha=.4)
    ax_pos.legend(fontsize=8)

    # 2) EE RPY (deg)
    t = t_buf['/ee_orientation_rpy']
    v = d_buf['/ee_orientation_rpy']
    for i, lbl in enumerate(LABELS['/ee_orientation_rpy']):
        ax_rpy.plot(t, [vv[i] for vv in v], label=lbl)
    # cart_goal 3-5
    if tg:
        for i, lbl in zip((3,4,5), ['yaw_goal°','pitch_goal°','roll_goal°']):
            ax_rpy.plot(tg, [g[i] for g in vg],
                        '--', color='k', linewidth=1, label=lbl)
    ax_rpy.set_title('ee_orientation_rpy')
    ax_rpy.grid(True, linestyle='--', alpha=.4)
    ax_rpy.legend(fontsize=8)

    # 3) cart_pos_err
    t = t_buf['/cart_pos_err']; v = d_buf['/cart_pos_err']
    for i, lbl in enumerate(LABELS['/cart_pos_err']):
        ax_err.plot(t, [vv[i] for vv in v], label=lbl)
    ax_err.set_title('cart_pos_err')
    ax_err.grid(True, linestyle='--', alpha=.4)
    ax_err.legend(fontsize=8)

    # 4) tau_total
    t = t_buf['/tau_total']; v = d_buf['/tau_total']
    for i, lbl in enumerate(LABELS['/tau_total']):
        ax_tau.plot(t, [vv[i] for vv in v], label=lbl)
    ax_tau.set_title('tau_total')
    ax_tau.grid(True, linestyle='--', alpha=.4)
    ax_tau.legend(fontsize=8)

    ax_err.set_xlabel('time [s]')
    ax_tau.set_xlabel('time [s]')
    fig.tight_layout()
    plt.show()


# ───── main ─────
def main():
    parser = argparse.ArgumentParser(
        description='Plot Franka bag with cart_goal overlay')
    parser.add_argument('bag_dir', nargs='?', default=DEFAULT_BAG_DIR,
                        help=f'bag directory (default: {DEFAULT_BAG_DIR})')
    args = parser.parse_args()

    print(f'▶ reading bag: {args.bag_dir}')
    t_buf, d_buf = load_topics(args.bag_dir)

    print('▶ plotting…')
    make_plot(t_buf, d_buf)


if __name__ == '__main__':
    main()
