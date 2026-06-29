#!/usr/bin/env python3

import argparse
import csv
import os
import select
import signal
import socket
import struct
import time
from collections import Counter
from datetime import datetime


CAN_FRAME = struct.Struct("=IB3x8s")
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x7FF

OUTPUT_TYPES = {
    0: "NONE",
    1: "OBJECTS",
    2: "CLUSTERS",
    3: "OUTPUT_ERROR",
}

EGO_MOTION_STATUS = {
    0: "INPUT_OK",
    1: "SPEED_MISSING",
    2: "YAW_MISSING",
    3: "SPEED_YAW_MISSING",
}

SORTING_MODES = {
    0: "NO_SORT",
    1: "BY_RANGE",
    2: "BY_RCS",
    3: "SORT_ERROR",
}

OBJECT_CLASSES = {
    0: "POINT",
    1: "CAR",
    2: "TRUCK",
    3: "RESERVED_01",
    4: "MOTORCYCLE",
    5: "BICYCLE",
    6: "WIDE",
    7: "RESERVED_02",
}

DYNAMIC_PROPERTIES = {
    0: "MOVING",
    1: "STATIONARY",
    2: "ONCOMING",
    3: "CROSSING_LEFT",
    4: "CROSSING_RIGHT",
    5: "UNKNOWN",
    6: "STOPPED",
}

EXISTENCE_PROBABILITY = {
    0: 0.0,
    1: 0.25,
    2: 0.5,
    3: 0.75,
    4: 0.9,
    5: 0.99,
    6: 0.999,
    7: 1.0,
}

FIELDNAMES = [
    "timestamp",
    "relative_time",
    "can_id",
    "dlc",
    "data_hex",
    "message",
    "output_type",
    "send_quality",
    "send_ext_info",
    "ego_motion_rx_status",
    "sensor_id",
    "sorting_mode",
    "max_distance_m",
    "persistent_error",
    "interference",
    "temperature_error",
    "temporary_error",
    "voltage_error",
    "num_objects",
    "num_clusters_near",
    "num_clusters_far",
    "measurement_counter",
    "interface_version",
    "object_id",
    "cluster_id",
    "x_m",
    "y_m",
    "vx_mps",
    "vy_mps",
    "rcs_dbm2",
    "dynamic_property",
    "existence_probability",
    "object_class",
    "orientation_deg",
    "length_m",
    "width_m",
    "acc_x_mps2",
    "acc_y_mps2",
    "false_alarm_probability",
    "invalid_state",
    "ambiguity_state",
    "version_raw",
]


def open_can(interface):
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((interface,))
    return sock


def recv_can(sock):
    frame = sock.recv(CAN_FRAME.size)
    can_id, dlc, data = CAN_FRAME.unpack(frame)
    return can_id & CAN_EFF_MASK, dlc, data[:dlc]


def parse_ids(value):
    if not value:
        return None
    return {int(item.strip(), 0) for item in value.split(",") if item.strip()}


def default_output_path():
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"ars408_can_{stamp}.csv"


def hex_id(can_id):
    return f"0x{can_id:X}"


def fmt_float(value):
    return f"{value:.3f}"


def decode_radar_state(row, data):
    if len(data) < 8:
        return
    row["message"] = "RADAR_STATE"
    row["max_distance_m"] = str(((((data[1] & 0xFF) << 2) + ((data[2] & 0xC0) >> 6)) << 1))
    row["persistent_error"] = str((data[2] & 0x20) >> 5)
    row["interference"] = str((data[2] & 0x10) >> 4)
    row["temperature_error"] = str((data[2] & 0x08) >> 3)
    row["temporary_error"] = str((data[2] & 0x04) >> 2)
    row["voltage_error"] = str((data[2] & 0x02) >> 1)
    row["sensor_id"] = str(data[4] & 0x07)
    row["sorting_mode"] = SORTING_MODES.get((data[4] & 0x70) >> 4, "")
    row["ego_motion_rx_status"] = EGO_MOTION_STATUS.get((data[5] & 0xC0) >> 6, "")
    row["send_ext_info"] = str((data[5] & 0x20) >> 5)
    row["send_quality"] = str((data[5] & 0x10) >> 4)
    row["output_type"] = OUTPUT_TYPES.get((data[5] & 0x0C) >> 2, "")


def decode_object_status(row, data):
    if len(data) < 4:
        return
    row["message"] = "OBJ_STATUS"
    row["num_objects"] = str(data[0])
    row["measurement_counter"] = str((data[2] << 8) + data[1])
    row["interface_version"] = str((data[3] & 0xF0) >> 4)


def decode_object_general(row, data):
    if len(data) < 8:
        return
    row["message"] = "OBJ_GENERAL"
    row["object_id"] = str(data[0])
    dist_x_tmp = (data[1] << 5) + ((data[2] & 0xF8) >> 3)
    dist_y_tmp = ((data[2] & 0x07) << 8) + data[3]
    speed_x_tmp = (data[4] << 2) + ((data[5] & 0xC0) >> 6)
    speed_y_tmp = ((data[5] & 0x3F) << 3) + ((data[6] & 0xE0) >> 5)
    row["x_m"] = fmt_float(dist_x_tmp * 0.2 - 500.0)
    row["y_m"] = fmt_float(dist_y_tmp * 0.2 - 204.6)
    row["vx_mps"] = fmt_float(speed_x_tmp * 0.25 - 128.0)
    row["vy_mps"] = fmt_float(speed_y_tmp * 0.25 - 64.0)
    row["dynamic_property"] = DYNAMIC_PROPERTIES.get(data[6] & 0x07, "")
    row["rcs_dbm2"] = fmt_float(data[7] * 0.5 - 64.0)


def decode_object_quality(row, data):
    if len(data) < 7:
        return
    row["message"] = "OBJ_QUALITY"
    row["object_id"] = str(data[0])
    prob_tmp = (data[6] & 0x1C) >> 2
    row["existence_probability"] = str(EXISTENCE_PROBABILITY.get(prob_tmp, ""))


def decode_object_extended(row, data):
    if len(data) < 8:
        return
    row["message"] = "OBJ_EXTENDED"
    row["object_id"] = str(data[0])
    acc_x_tmp = (data[1] << 3) + ((data[2] & 0xE0) >> 5)
    acc_y_tmp = ((data[2] & 0x1F) << 4) + ((data[3] & 0xF0) >> 4)
    angle_tmp = (data[4] << 2) + ((data[5] & 0xC0) >> 6)
    row["acc_x_mps2"] = fmt_float(acc_x_tmp * 0.01 - 10.0)
    row["acc_y_mps2"] = fmt_float(acc_y_tmp * 0.01 - 2.5)
    row["object_class"] = OBJECT_CLASSES.get(data[3] & 0x07, "")
    row["orientation_deg"] = fmt_float(angle_tmp * 0.4 - 180.0)
    row["length_m"] = fmt_float(data[6] * 0.2)
    row["width_m"] = fmt_float(data[7] * 0.2)


def decode_cluster_status(row, data):
    if len(data) < 5:
        return
    row["message"] = "CLUSTER_STATUS"
    row["num_clusters_near"] = str(data[0])
    row["num_clusters_far"] = str(data[1])
    row["measurement_counter"] = str((data[2] << 8) + data[3])
    row["interface_version"] = str((data[4] & 0xF0) >> 4)


def decode_cluster_general(row, data):
    if len(data) < 8:
        return
    row["message"] = "CLUSTER_GENERAL"
    row["cluster_id"] = str(data[0])
    dist_x_tmp = (data[1] << 5) + ((data[2] & 0xF8) >> 3)
    dist_y_tmp = ((data[2] & 0x03) << 8) + data[3]
    speed_x_tmp = (data[4] << 2) + ((data[5] & 0xC0) >> 6)
    speed_y_tmp = ((data[5] & 0x3F) << 3) + ((data[6] & 0xE0) >> 5)
    row["x_m"] = fmt_float(dist_x_tmp * 0.2 - 500.0)
    row["y_m"] = fmt_float(dist_y_tmp * 0.2 - 102.4)
    row["vx_mps"] = fmt_float(speed_x_tmp * 0.25 - 128.0)
    row["vy_mps"] = fmt_float(speed_y_tmp * 0.25 - 64.0)
    row["dynamic_property"] = DYNAMIC_PROPERTIES.get(data[6] & 0x07, "")
    row["rcs_dbm2"] = fmt_float(data[7] * 0.5 - 64.0)


def decode_cluster_quality(row, data):
    if len(data) < 5:
        return
    row["message"] = "CLUSTER_QUALITY"
    row["cluster_id"] = str(data[0])
    row["false_alarm_probability"] = str(data[3] & 0x07)
    row["ambiguity_state"] = str(data[4] & 0x07)
    row["invalid_state"] = str((data[4] & 0xF8) >> 3)


def decode_version(row, data):
    row["message"] = "VERSION_ID"
    row["version_raw"] = data.hex().upper()


DECODERS = {
    0x201: decode_radar_state,
    0x60A: decode_object_status,
    0x60B: decode_object_general,
    0x60C: decode_object_quality,
    0x60D: decode_object_extended,
    0x600: decode_cluster_status,
    0x701: decode_cluster_general,
    0x702: decode_cluster_quality,
    0x700: decode_version,
}


def write_summary(path, counts, first_seen, last_seen):
    with open(path, "w", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=["can_id", "count", "first_timestamp", "last_timestamp", "approx_hz"],
        )
        writer.writeheader()
        for can_id in sorted(counts):
            duration = max(0.0, last_seen[can_id] - first_seen[can_id])
            hz = counts[can_id] / duration if duration > 0.0 else 0.0
            writer.writerow(
                {
                    "can_id": hex_id(can_id),
                    "count": counts[can_id],
                    "first_timestamp": f"{first_seen[can_id]:.6f}",
                    "last_timestamp": f"{last_seen[can_id]:.6f}",
                    "approx_hz": f"{hz:.3f}",
                }
            )


def main():
    parser = argparse.ArgumentParser(description="Record ARS408 CAN frames to CSV.")
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--output", default=default_output_path())
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 means until Ctrl-C")
    parser.add_argument(
        "--ids",
        default="",
        help="comma separated CAN IDs to keep, e.g. 0x201,0x700,0x60A; empty records all",
    )
    parser.add_argument("--summary-output", default="", help="default: <output>_summary.csv")
    parser.add_argument("--print-period", type=float, default=1.0)
    args = parser.parse_args()

    keep_ids = parse_ids(args.ids)
    output_path = os.path.abspath(args.output)
    summary_path = (
        os.path.abspath(args.summary_output)
        if args.summary_output
        else os.path.splitext(output_path)[0] + "_summary.csv"
    )

    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    sock = open_can(args.interface)
    counts = Counter()
    first_seen = {}
    last_seen = {}
    start_time = time.time()
    last_print = start_time

    with open(output_path, "w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=FIELDNAMES)
        writer.writeheader()

        print(f"recording {args.interface} to {output_path}")
        if keep_ids:
            print("filter ids: " + ", ".join(hex_id(can_id) for can_id in sorted(keep_ids)))

        while running:
            if args.duration > 0.0 and time.time() - start_time >= args.duration:
                break

            readable, _, _ = select.select([sock], [], [], 0.2)
            if not readable:
                continue

            timestamp = time.time()
            can_id, dlc, data = recv_can(sock)
            base_id = can_id & CAN_SFF_MASK
            if keep_ids is not None and base_id not in keep_ids:
                continue

            row = {field: "" for field in FIELDNAMES}
            row["timestamp"] = f"{timestamp:.6f}"
            row["relative_time"] = f"{timestamp - start_time:.6f}"
            row["can_id"] = hex_id(base_id)
            row["dlc"] = str(dlc)
            row["data_hex"] = data.hex().upper()

            decoder = DECODERS.get(base_id)
            if decoder:
                decoder(row, data)

            writer.writerow(row)
            counts[base_id] += 1
            first_seen.setdefault(base_id, timestamp)
            last_seen[base_id] = timestamp

            if args.print_period > 0.0 and timestamp - last_print >= args.print_period:
                total = sum(counts.values())
                print(f"recorded {total} frames, ids={len(counts)}")
                last_print = timestamp

    write_summary(summary_path, counts, first_seen, last_seen)
    print(f"wrote {output_path}")
    print(f"wrote {summary_path}")


if __name__ == "__main__":
    main()
