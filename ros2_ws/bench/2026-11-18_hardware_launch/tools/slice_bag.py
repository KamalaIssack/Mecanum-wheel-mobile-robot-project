#!/usr/bin/env python3
"""Copy a time window of a rosbag2 bag into a new MCAP bag.

Usage:
    python3 slice_bag.py <in_bag> <out_bag> <start> <end>

<start> and <end> are ISO 8601 times with a UTC offset, for example
2026-11-18T17:51:51.000+03:00. The window uses bag receive times and
includes both ends. Messages are copied as raw serialized bytes (no
deserialisation), and every topic of the input bag is kept with its
original type and QoS. <out_bag> must not exist yet.

The slices in ../data were made with (from the repo root, ROS sourced):
    T=ros2_ws/bench/2026-11-18_hardware_launch
    python3 $T/tools/slice_bag.py _incoming/e4_demo_take1 $T/data/run_1751 \\
        2026-11-18T17:51:51.000+03:00 2026-11-18T17:52:18.000+03:00
    python3 $T/tools/slice_bag.py _incoming/e4_demo_take1 $T/data/watchdog_trip_1813 \\
        2026-11-18T18:13:16.675+03:00 2026-11-18T18:13:26.876+03:00
    python3 $T/tools/slice_bag.py _incoming/e4_demo_take1 $T/data/watchdog_trip_1815 \\
        2026-11-18T18:15:48.039+03:00 2026-11-18T18:15:58.033+03:00
"""
import sys
from collections import Counter
from datetime import datetime

import rosbag2_py


def to_ns(text):
    t = datetime.fromisoformat(text)
    if t.tzinfo is None:
        sys.exit(f'{text}: give a UTC offset, for example +03:00')
    return round(t.timestamp() * 1e9)


def main():
    if len(sys.argv) != 5:
        sys.exit(__doc__)
    in_bag, out_bag = sys.argv[1], sys.argv[2]
    start, end = to_ns(sys.argv[3]), to_ns(sys.argv[4])
    if end <= start:
        sys.exit('end must be after start')

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=in_bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=out_bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for topic in reader.get_all_topics_and_types():
        writer.create_topic(topic)

    reader.seek(start)
    counts = Counter()
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if stamp > end:
            break
        if stamp >= start:
            writer.write(topic, data, stamp)
            counts[topic] += 1
    del writer  # closes and finalises the bag

    for topic, n in sorted(counts.items()):
        print(f'{topic}: {n}')


if __name__ == '__main__':
    main()
