#!/usr/bin/env python3
"""
plot_franka_bag.py
─────────────────────────────────────────────────────────────
네 토픽을 2×2 subplot 한 Figure 로 플롯.
 - 기본 BAG 경로(DEFAULT_BAG_DIR) 지정
 - 인자를 주면 인자 > 기본값 순으로 사용
─────────────────────────────────────────────────────────────
"""
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ★★★ 여기서 기본 BAG 디렉토리를 지정하세요 ★★★
DEFAULT_BAG_DIR = '/home/airplon/franka_ws/bagfiles/force_pd'
# DEFAULT_BAG_DIR = '/home/airplon/franka_ws/franka_example_controllers/bagfiles/force_pd'

# 관심 토픽
TOPICS = [
    '/ee_position',
    '/ee_orientation_rpy',
    '/cart_pos_err',
    '/tau_total',
]

LABELS = {
    '/ee_position':        ['x', 'y', 'z'],
    '/ee_orientation_rpy': ['yaw(Z)', 'pitch(Y)', 'roll(X)'],
    '/cart_pos_err':       [f'err{i}' for i in range(6)],
    '/tau_total':          [f'τ{i}'   for i in range(7)],
}


# ───────────────────── bag 로딩 ──────────────────────
def load_topics(bag_dir, topics):
    bag_dir = Path(bag_dir).expanduser().resolve()
    if not bag_dir.is_dir():
        raise FileNotFoundError(bag_dir)

    storage_opts   = rosbag2_py._storage.StorageOptions(
        uri=str(bag_dir), storage_id='sqlite3')
    converter_opts = rosbag2_py._storage.ConverterOptions('', 'cdr')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_opts, converter_opts)

    type_map = {info.name: info.type
                for info in reader.get_all_topics_and_types()}

    t_buf   = {t: [] for t in topics}
    data_buf = {t: [] for t in topics}

    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        if topic not in topics:
            continue

        msg_cls = get_message(type_map[topic])
        msg     = deserialize_message(raw, msg_cls)

        t_buf[topic].append(stamp_ns * 1e-9)  # ns → s

        if topic == '/ee_position':
            data_buf[topic].append(
                (msg.point.x, msg.point.y, msg.point.z))
        else:
            data_buf[topic].append(list(msg.data))

    return t_buf, data_buf


# ───────────────────── 플롯 ──────────────────────
def make_comparison_plot(t_buf, data_buf):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex='col')
    axes = axes.flatten()

    for ax, topic in zip(axes, TOPICS):
        t = t_buf[topic]
        v = data_buf[topic]
        if not t:
            ax.set_title(f'{topic} (no data)')
            continue

        for idx, label in enumerate(LABELS[topic]):
            ax.plot(t, [val[idx] for val in v], label=label)

        ax.set_title(topic.lstrip('/'))
        ax.grid(True, linestyle='--', alpha=.4)
        ax.legend(fontsize=8, loc='best')

    axes[2].set_xlabel('time [s]')
    axes[3].set_xlabel('time [s]')
    fig.tight_layout()
    plt.show()


# ───────────────────── main ──────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='Plot Franka bag variables (ee pos/orient, cart err, τ)')
    parser.add_argument(
        'bag_dir', nargs='?',
        default=DEFAULT_BAG_DIR,
        help='bag directory (metadata.yaml & *.db3). '
             f'Default: {DEFAULT_BAG_DIR}')
    args = parser.parse_args()

    print(f'▶ reading bag: {args.bag_dir}')
    t_buf, data_buf = load_topics(args.bag_dir, TOPICS)

    print('▶ plotting…')
    make_comparison_plot(t_buf, data_buf)


if __name__ == '__main__':
    main()
