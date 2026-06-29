#!/usr/bin/env python3

import argparse
import socket
import struct
import time


CAN_FRAME = struct.Struct("=IB3x8s")


def str_to_bool(value):
    value = value.lower()
    if value in ("1", "true", "yes", "on"):
        return True
    if value in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value}")


def open_can(interface):
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((interface,))
    return sock


def send_can(sock, can_id, payload):
    padded = payload.ljust(8, b"\x00")
    sock.send(CAN_FRAME.pack(can_id, len(payload), padded))


def build_radar_cfg(mode, quality, ext_info):
    data = [0] * 8

    # UpdateOutputType
    data[0] |= 0x08
    if mode == "none":
        data[4] |= 0x00
    elif mode == "objects":
        data[4] |= 0x08
    elif mode == "clusters":
        data[4] |= 0x10
    else:
        raise ValueError(f"unsupported mode: {mode}")

    # UpdateSendQuality
    data[0] |= 0x10
    if quality:
        data[5] |= 0x04

    # Extended info is meaningful for object output only.
    if mode == "objects":
        data[0] |= 0x20
        if ext_info:
            data[5] |= 0x08

    return bytes(data)


def main():
    parser = argparse.ArgumentParser(description="Configure Continental ARS408 output mode.")
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--mode", choices=("objects", "clusters", "none"), default="objects")
    parser.add_argument("--quality", type=str_to_bool, default=True)
    parser.add_argument("--ext-info", type=str_to_bool, default=True)
    parser.add_argument("--delay-sec", type=float, default=0.5)
    parser.add_argument("--can-id", type=lambda x: int(x, 0), default=0x200)
    args, _unknown_ros_args = parser.parse_known_args()

    if args.delay_sec > 0.0:
        time.sleep(args.delay_sec)

    payload = build_radar_cfg(args.mode, args.quality, args.ext_info)
    sock = open_can(args.interface)
    send_can(sock, args.can_id, payload)

    print(f"sent {args.interface} {args.can_id:X}#{payload.hex().upper()} mode={args.mode}")


if __name__ == "__main__":
    main()
