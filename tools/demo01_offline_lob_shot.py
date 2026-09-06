#!/usr/bin/env python3
"""重装吊射第一个离线 demo。

不连接电控、ROS、雷达、相机、192.168.8.1。
对照只读源码：
  - hero 串口 packet.hpp / solve_angle（z=0/1/2）
  - hero_lidar_lob_shot 弹道（只解低抛，且 fire_flag=1）
  - 自定义客户端 linuxsender1（0x0310，每包最多 298 字节 H264）
  - RM2027 预告：42mm 初速上限 16.5 m/s；装配时发射机构断电
"""

from __future__ import annotations

import math
import struct
import sys

G = 9.7915
V0 = 16.5  # 2027 重装解锁后初速上限，性能体系统一
TUNNEL_H = 0.27  # 2027 隧道外廓高度，单位 m
TARGET = (23.125, 1.510, 0.840)  # 沿用 ITL/本队英雄吊射先验基地坐标
MUZZLE = (3.00, 1.50, 0.40)

# 与 hero/src/jlcv_serial_driver/src/crc.cpp 同一张表
CRC16_INIT = 0xFFFF
W_CRC_TABLE = [
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
]

SEND_HEADER_ANGLE = 0xA5
CMD_CUSTOM_TO_CLIENT = 0x0310
SERIAL_PKT_SIZE = 300
MAX_CHUNK_DATA = 298

# 与 solve_angle 一致，尚未与电控联调吊射，demo 只保证编码合同正确
CTRL_IDLE = 0
CTRL_AIM = 1
CTRL_FIRE = 2


class CheckFailure(Exception):
    pass


def check(cond: bool, msg: str) -> None:
    if not cond:
        raise CheckFailure(msg)


def crc16_append(payload_without_crc: bytes) -> bytes:
    crc = CRC16_INIT
    for ch in payload_without_crc:
        crc = (crc >> 8) ^ W_CRC_TABLE[(crc ^ ch) & 0xFF]
    return payload_without_crc + struct.pack("<H", crc)


def vacuum_pitches(x: float, y: float, v: float = V0, g: float = G) -> tuple[float, float]:
    """真空弹道闭式解。low 为平射支，high 为吊射支。"""
    v2 = v * v
    disc = v2 * v2 - g * (g * x * x + 2.0 * y * v2)
    check(disc >= 0.0, f"无实数弹道解: x={x:.3f} y={y:.3f} v={v}")
    root = math.sqrt(disc)
    low = math.atan((v2 - root) / (g * x))
    high = math.atan((v2 + root) / (g * x))
    return low, high


def apex_height(pitch: float, v: float = V0, g: float = G) -> float:
    vy = v * math.sin(pitch)
    return (vy * vy) / (2.0 * g)


def encode_gimbal_send(pitch: float, yaw: float, z: int, deploy: int = 0) -> bytes:
    """对照 packet.hpp SendPacket：0xA5 + float pitch + float yaw + u8 z + u8 deploy + crc16。"""
    check(z in (CTRL_IDLE, CTRL_AIM, CTRL_FIRE), f"非法 control_status={z}")
    body = struct.pack("<BffBB", SEND_HEADER_ANGLE, pitch, yaw, z, deploy)
    packet = crc16_append(body)
    check(len(packet) == 13, f"SendPacket 长度应为 13，实际 {len(packet)}")
    return packet


def pack_0310_chunk(h264: bytes) -> bytes:
    """对照自定义客户端：300 字节 = uint16 长度 + 最多 298 字节数据 + padding。"""
    check(len(h264) <= MAX_CHUNK_DATA, f"H264 chunk 超长 {len(h264)}")
    payload = struct.pack("<H", len(h264)) + h264
    payload = payload + b"\x00" * (SERIAL_PKT_SIZE - len(payload))
    check(len(payload) == SERIAL_PKT_SIZE, "0x0310 payload 必须正好 300 字节")
    return payload


def module_fsm(unlocked: bool, docked: bool, assembling: bool, trigger: bool) -> str:
    """重装发射模块生命周期。装配中发射机构断电，禁止进入 READY。"""
    if assembling:
        return "FIRE_INHIBITED"
    if not unlocked:
        return "LOCKED_WAIT"
    if not docked:
        return "WAITING_DOCK"
    if not trigger:
        return "IDLE"
    return "CAN_AIM"


def lidar_camera_feasibility() -> list[str]:
    return [
        "可行且推荐做的是外参标定后的「投影融合」，不是当场重建整场三维网格。",
        "导航栈已有 MID360：/livox/lidar + Faster-LIO/ICP → map→base_link。吊射主链路应吃这份位姿，不要另起一套 SLAM。",
        "海康是 2D 工业相机（vision_bringup camera_info，约 100fps）。它提供纹理/绿灯/装甲外观，不提供度量深度。",
        "有标定后：图像像素 ←K, T_cam_lidar→ 点云射线，可给基地引导灯、装甲贴颜色，或把视觉检测点落到 map 系。",
        "稠密三维重建（TSDF/NeRF/实时网格）在赛场周期内不划算：MID360 稀疏、车在动、算力还要给自瞄和导航。",
        "超分属于操作间自定义客户端（已部署 192.168.8.1），只改善人眼看图，不提高弹道度量精度。",
        "RM2026 自定义数据 0x0310 单包 300 字节、有效 298 字节，走裁判串口图传，不是以太网整帧回传。",
        "因此 demo 之后的落地顺序：定位 TF → 高抛解算 → 串口 z=0/1/2 与电控对表 → 再考虑相机投影辅助，最后才是超分画面。",
    ]


def run() -> None:
    dx = TARGET[0] - MUZZLE[0]
    dy = TARGET[1] - MUZZLE[1]
    dz = TARGET[2] - MUZZLE[2]
    x_horiz = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)

    low, high = vacuum_pitches(x_horiz, dz, V0)
    high_apex = apex_height(high)
    low_apex = apex_height(low)

    print("=== demo01 重装吊射离线 ===")
    print(f"初速 {V0} m/s  水平距 {x_horiz:.3f} m  高差 {dz:.3f} m")
    print(f"低抛 {math.degrees(low):.2f}°  顶点 {low_apex:.2f} m")
    print(f"高抛 {math.degrees(high):.2f}°  顶点 {high_apex:.2f} m  yaw {math.degrees(yaw):.2f}°")

    check(high > math.radians(45.0), "吊射支应大于 45°")
    check(low < math.radians(45.0), "平射支应小于 45°")
    check(high_apex > TUNNEL_H + 0.5, "高抛应明显越过 270mm 隧道外廓")
    check(low_apex < high_apex, "低抛顶点应低于高抛")
    # 现有 hero_lidar_lob_shot 取 (v2 - sqrt)，即低抛；吊射不能沿用
    check(abs(high - low) > math.radians(10.0), "高/低两支必须可区分")

    check(module_fsm(False, False, False, True) == "LOCKED_WAIT", "未解锁应等待")
    check(module_fsm(True, False, False, True) == "WAITING_DOCK", "未对接应等待安装")
    check(module_fsm(True, True, True, True) == "FIRE_INHIBITED", "装配中必须禁射")
    check(module_fsm(True, True, False, True) == "CAN_AIM", "解锁且对接且非装配才可瞄准")

    idle = encode_gimbal_send(0.0, 0.0, CTRL_IDLE)
    aim = encode_gimbal_send(high, yaw, CTRL_AIM)
    fire = encode_gimbal_send(high, yaw, CTRL_FIRE)
    check(idle[0] == SEND_HEADER_ANGLE and idle[9] == CTRL_IDLE, "空闲帧 z 应为 0")
    check(aim[9] == CTRL_AIM, "瞄准帧 z 应为 1，不能写成 hero_lidar 的 fire_ready=1 当开火")
    check(fire[9] == CTRL_FIRE, "开火帧 z 应为 2（与 solve_angle 一致）")
    print(f"串口 0xA5 空闲 {idle.hex()}")
    print(f"串口 0xA5 瞄准 {aim.hex()}")
    print(f"串口 0xA5 开火 {fire.hex()}")
    print("说明：吊射尚未与电控联调。这里只固定与自瞄同一套 0/1/2 合同，不改串口驱动。")

    chunk = pack_0310_chunk(b"\x00\x00\x00\x01nalu-demo")
    h264_len = struct.unpack_from("<H", chunk, 0)[0]
    check(h264_len == 13, "演示 chunk 长度字段")
    check(chunk[2:2 + h264_len].startswith(b"\x00\x00\x00\x01"), "H264 起始码应落在长度之后")
    print(f"图传 0x0310 单包 {len(chunk)} B，有效 {h264_len} B（上限 {MAX_CHUNK_DATA}）")
    print("说明：赛场无整帧图传回操作间时，沿用车辆串口自定义数据转发；超分只在 192.168.8.1 客户端侧。")

    print("=== 雷达+海康三维重建可行性 ===")
    for line in lidar_camera_feasibility():
        print(f"- {line}")

    print("DEMO01 PASS")


if __name__ == "__main__":
    try:
        run()
    except CheckFailure as exc:
        print(f"DEMO01 FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
    sys.exit(0)
