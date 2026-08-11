#!/usr/bin/env python3
"""Repair incomplete rosbag2 sqlite3 bags after abnormal shutdown (Foxy has no reindex)."""

import argparse
import os
import sqlite3
import sys


def find_db3(bag_dir):
    for name in sorted(os.listdir(bag_dir)):
        if name.endswith(".db3") and not name.endswith(".db3-wal"):
            return name
    return None


def checkpoint_sqlite(db_path):
    conn = sqlite3.connect(db_path)
    conn.execute("PRAGMA wal_checkpoint(TRUNCATE)")
    conn.commit()
    conn.close()


def remove_wal_shm(bag_dir, db_name):
    for ext in ("-wal", "-shm"):
        path = os.path.join(bag_dir, db_name + ext)
        if os.path.exists(path):
            os.remove(path)


def yaml_string(value):
    if not value:
        return '""'
    escaped = value.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
    return f'"{escaped}"'


def generate_metadata(bag_dir, db_name):
    db_path = os.path.join(bag_dir, db_name)
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute(
        "SELECT id, name, type, serialization_format, offered_qos_profiles "
        "FROM topics ORDER BY id"
    )
    topics = cur.fetchall()
    cur.execute("SELECT COUNT(*), MIN(timestamp), MAX(timestamp) FROM messages")
    total, tmin, tmax = cur.fetchone()
    total = total or 0
    if total == 0:
        duration = 0
        start = 9223372036854775807
    else:
        duration = (tmax - tmin) if tmax is not None and tmin is not None else 0
        start = tmin

    lines = [
        "rosbag2_bagfile_information:",
        "  version: 4",
        "  storage_identifier: sqlite3",
        "  relative_file_paths:",
        f"    - {db_name}",
        "  duration:",
        f"    nanoseconds: {duration}",
        "  starting_time:",
        f"    nanoseconds_since_epoch: {start}",
        f"  message_count: {total}",
        "  topics_with_message_count:",
    ]
    for topic_id, name, typ, ser_fmt, qos in topics:
        cur.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (topic_id,))
        (count,) = cur.fetchone()
        lines.extend(
            [
                "    - topic_metadata:",
                f"        name: {name}",
                f"        type: {typ}",
                f"        serialization_format: {ser_fmt}",
                f"        offered_qos_profiles: {yaml_string(qos or '')}",
                f"      message_count: {count}",
            ]
        )
    lines.extend(['  compression_format: ""', '  compression_mode: ""', ""])
    conn.close()

    meta_path = os.path.join(bag_dir, "metadata.yaml")
    with open(meta_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))


def repair_bag(bag_dir):
    bag_dir = os.path.abspath(bag_dir)
    if not os.path.isdir(bag_dir):
        raise RuntimeError(f"not a directory: {bag_dir}")

    meta_path = os.path.join(bag_dir, "metadata.yaml")
    if os.path.exists(meta_path):
        return True

    db_name = find_db3(bag_dir)
    if not db_name:
        raise RuntimeError("no .db3 database found")

    db_path = os.path.join(bag_dir, db_name)
    checkpoint_sqlite(db_path)
    remove_wal_shm(bag_dir, db_name)
    generate_metadata(bag_dir, db_name)
    return os.path.exists(meta_path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_dir", help="Path to rosbag2 bag directory")
    args = parser.parse_args()
    try:
        ok = repair_bag(args.bag_dir)
    except Exception as exc:
        print(f"repair_rosbag2: {exc}", file=sys.stderr)
        return 1
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
