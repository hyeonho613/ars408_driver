#!/usr/bin/env python3

import argparse
import select
import signal
import socket
import struct
import time


CAN_FRAME = struct.Struct("=IB3x8s")
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x7FF


def open_can(interface):
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((interface,))
    return sock


def recv_can(sock):
    frame = sock.recv(CAN_FRAME.size)
    can_id, dlc, data = CAN_FRAME.unpack(frame)
    return can_id & CAN_EFF_MASK, dlc, data[:dlc]


def send_can(sock, can_id, payload):
    if len(payload) > 8:
        raise ValueError("CAN payload must be 8 bytes or less")
    padded = payload.ljust(8, b"\x00")
    sock.send(CAN_FRAME.pack(can_id, len(payload), padded))


def encode_ars408_speed(speed_mps, forward_direction, reverse_direction, stop_threshold_mps):
    speed = abs(speed_mps)
    raw_speed = int(round(speed / 0.02))
    raw_speed = max(0, min(raw_speed, 8190))

    if speed < stop_threshold_mps:
        direction = 0
    elif speed_mps >= 0:
        direction = forward_direction
    else:
        direction = reverse_direction

    # ARS408 SpeedInformation packs a 13-bit speed raw value followed by direction bits.
    byte0 = (raw_speed >> 5) & 0xFF
    byte1 = ((raw_speed & 0x1F) << 3) | (direction & 0x07)
    return bytes([byte0, byte1])


def encode_ars408_yaw_rate(yaw_rate_deg_s):
    raw_yaw = int(round((yaw_rate_deg_s + 327.68) / 0.01))
    raw_yaw = max(0, min(raw_yaw, 65535))
    return bytes([(raw_yaw >> 8) & 0xFF, raw_yaw & 0xFF])


def main():
    parser = argparse.ArgumentParser(
        description="Forward Kia Niro 0x52A vehicle speed to Continental ARS408 ego-motion CAN input."
    )
    parser.add_argument("--input-interface", default="can0")
    parser.add_argument("--output-interface", default="can2")
    parser.add_argument("--niro-speed-id", type=lambda x: int(x, 0), default=0x52A)
    parser.add_argument("--ars-speed-id", type=lambda x: int(x, 0), default=0x300)
    parser.add_argument("--ars-yaw-id", type=lambda x: int(x, 0), default=0x301)
    parser.add_argument("--niro-speed-byte", type=int, default=0)
    parser.add_argument("--niro-speed-scale", type=float, default=0.3, help="m/s per raw count")
    parser.add_argument("--yaw-rate-deg-s", type=float, default=0.0)
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--stop-threshold-mps", type=float, default=0.05)
    parser.add_argument("--forward-direction", type=int, default=1)
    parser.add_argument("--reverse-direction", type=int, default=2)
    parser.add_argument("--print-period", type=float, default=1.0)
    args = parser.parse_args()

    if not 0 <= args.niro_speed_byte <= 7:
        raise ValueError("--niro-speed-byte must be in range 0..7")

    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    rx = open_can(args.input_interface)
    tx = open_can(args.output_interface)

    last_speed_mps = 0.0
    last_send = 0.0
    last_print = 0.0

    print(
        "Listening for Kia Niro speed on "
        f"{args.input_interface} id=0x{args.niro_speed_id:X}; "
        f"sending ARS408 ego-motion on {args.output_interface} "
        f"speed id=0x{args.ars_speed_id:X}, yaw id=0x{args.ars_yaw_id:X}"
    )

    while running:
        timeout = max(0.0, args.period - (time.monotonic() - last_send))
        readable, _, _ = select.select([rx], [], [], timeout)

        for sock in readable:
            can_id, dlc, data = recv_can(sock)
            if (can_id & CAN_SFF_MASK) != args.niro_speed_id or dlc <= args.niro_speed_byte:
                continue
            last_speed_mps = data[args.niro_speed_byte] * args.niro_speed_scale

        now = time.monotonic()
        if now - last_send >= args.period:
            speed_payload = encode_ars408_speed(
                last_speed_mps,
                args.forward_direction,
                args.reverse_direction,
                args.stop_threshold_mps,
            )
            yaw_payload = encode_ars408_yaw_rate(args.yaw_rate_deg_s)
            send_can(tx, args.ars_speed_id, speed_payload)
            send_can(tx, args.ars_yaw_id, yaw_payload)
            last_send = now

        if now - last_print >= args.print_period:
            speed_payload = encode_ars408_speed(
                last_speed_mps,
                args.forward_direction,
                args.reverse_direction,
                args.stop_threshold_mps,
            )
            yaw_payload = encode_ars408_yaw_rate(args.yaw_rate_deg_s)
            print(
                f"speed={last_speed_mps:.3f} m/s "
                f"({last_speed_mps * 3.6:.2f} km/h) "
                f"300#{speed_payload.hex().upper()} "
                f"301#{yaw_payload.hex().upper()}"
            )
            last_print = now


if __name__ == "__main__":
    main()
