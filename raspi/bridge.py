#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import time
import errno
from dataclasses import asdict, dataclass
from typing import Any, Optional

try:
    from smbus2 import SMBus, i2c_msg
except ImportError as exc:  # pragma: no cover
    raise SystemExit("缺少依赖 smbus2，请先安装 requirements.txt") from exc

try:
    import websockets
    from websockets.exceptions import ConnectionClosed
except ImportError as exc:  # pragma: no cover
    raise SystemExit("缺少依赖 websockets，请先安装 requirements.txt") from exc

SERVO_ID_MIN = 21
SERVO_ID_MAX = 43
SERVO_COUNT = SERVO_ID_MAX - SERVO_ID_MIN + 1

FRAME_MAGIC = b"\xAA\x55"
FRAME_VERSION = 1
FRAME_SIZE = 60
FRAME_HEADER_SIZE = 12
CMD_GET_FRAME = 0xA5


@dataclass
class BridgeConfig:
    i2c_bus: int = 1
    i2c_addr: int = 0x28
    i2c_request_gap_ms: int = 2
    i2c_poll_interval_ms: int = 20
    i2c_retry_delay_ms: int = 200

    ws_uri: str = "ws://192.168.0.100:9102/"
    ws_client_name: str = "rpi-body"
    ws_preferred_target_name: str = ""
    ws_reconnect_delay_ms: int = 2000
    ws_heartbeat_interval_ms: int = 15000

    send_interval_ms: int = 30
    send_min_delta_deg: int = 2
    send_batch_max: int = 8
    send_max_batch_per_tick: int = 1

    filter_alpha: float = 0.22
    filter_deadzone_deg: int = 5
    filter_max_step_per_tick: int = 6


class ServoOutputFilter:
    def __init__(self, alpha: float, deadzone: int, max_step: int, count: int) -> None:
        self.alpha = alpha
        self.deadzone = deadzone
        self.max_step = max_step
        self.filtered = [0.0] * count
        self.stable = [0] * count
        self.inited = [False] * count

    def apply(self, idx: int, raw_angle: float) -> int:
        if not self.inited[idx]:
            out = clamp_int(int(round(raw_angle)), 0, 360)
            self.filtered[idx] = raw_angle
            self.stable[idx] = out
            self.inited[idx] = True
            return out

        self.filtered[idx] += self.alpha * (raw_angle - self.filtered[idx])
        candidate = clamp_int(int(round(self.filtered[idx])), 0, 360)
        delta = candidate - self.stable[idx]
        abs_delta = abs(delta)

        if abs_delta <= self.deadzone:
            return self.stable[idx]

        if delta > self.max_step:
            delta = self.max_step
        elif delta < -self.max_step:
            delta = -self.max_step

        self.stable[idx] = clamp_int(self.stable[idx] + delta, 0, 360)
        return self.stable[idx]


class Esp32I2CClient:
    def __init__(self, bus_id: int, slave_addr: int, request_gap_ms: int) -> None:
        self.bus = SMBus(bus_id)
        self.addr = slave_addr
        self.request_gap_sec = max(0, request_gap_ms) / 1000.0

    def read_frame(self) -> bytes:
        self.bus.write_byte(self.addr, CMD_GET_FRAME)
        if self.request_gap_sec > 0:
            time.sleep(self.request_gap_sec)

        msg = i2c_msg.read(self.addr, FRAME_SIZE)
        self.bus.i2c_rdwr(msg)
        return bytes(msg)

    def close(self) -> None:
        self.bus.close()


class RaspiWsBridge:
    def __init__(self, cfg: BridgeConfig) -> None:
        self.cfg = cfg
        self.i2c_client = Esp32I2CClient(cfg.i2c_bus, cfg.i2c_addr, cfg.i2c_request_gap_ms)
        self.servo_filter = ServoOutputFilter(
            alpha=cfg.filter_alpha,
            deadzone=cfg.filter_deadzone_deg,
            max_step=cfg.filter_max_step_per_tick,
            count=SERVO_COUNT,
        )

        self.latest_seq = 0
        self.latest_uptime_ms = 0
        self.latest_angles: Optional[list[int]] = None

        self.ws_self_id = ""
        self.ws_target_id = ""
        self.need_full_sync = True

        self.last_sent_angles = [0] * SERVO_COUNT
        self.last_sent_valid = [False] * SERVO_COUNT

        self.state_lock = asyncio.Lock()
        self.last_wait_bind_log = 0.0
        self.last_no_change_log = 0.0
        self.last_send_throttle_log = 0.0

    async def run(self) -> None:
        poll_task = asyncio.create_task(self.i2c_poll_loop(), name="i2c_poll")
        try:
            await self.ws_loop()
        finally:
            poll_task.cancel()
            await asyncio.gather(poll_task, return_exceptions=True)
            self.i2c_client.close()

    async def i2c_poll_loop(self) -> None:
        poll_interval = self.cfg.i2c_poll_interval_ms / 1000.0
        retry_interval = self.cfg.i2c_retry_delay_ms / 1000.0

        while True:
            try:
                frame = await asyncio.to_thread(self.i2c_client.read_frame)
                parsed = parse_adc_frame(frame)
                if parsed is None:
                    await asyncio.sleep(retry_interval)
                    continue

                seq, uptime_ms, raw_angles = parsed
                filtered = [self.servo_filter.apply(i, raw_angles[i]) for i in range(SERVO_COUNT)]

                async with self.state_lock:
                    self.latest_seq = seq
                    self.latest_uptime_ms = uptime_ms
                    self.latest_angles = filtered
            except OSError as exc:
                if exc.errno == errno.EREMOTEIO:
                    logging.warning(
                        "I2C读取失败(Remote I/O): bus=%d addr=%d(0x%02x), 请检查i2c_addr是否匹配(0x28=40)",
                        self.cfg.i2c_bus,
                        self.cfg.i2c_addr,
                        self.cfg.i2c_addr,
                    )
                else:
                    logging.warning(
                        "I2C读取失败: bus=%d addr=%d(0x%02x), err=%s",
                        self.cfg.i2c_bus,
                        self.cfg.i2c_addr,
                        self.cfg.i2c_addr,
                        exc,
                    )
                await asyncio.sleep(retry_interval)
                continue
            except Exception as exc:
                logging.warning(
                    "I2C读取失败: bus=%d addr=%d(0x%02x), err=%s",
                    self.cfg.i2c_bus,
                    self.cfg.i2c_addr,
                    self.cfg.i2c_addr,
                    exc,
                )
                await asyncio.sleep(retry_interval)
                continue

            await asyncio.sleep(poll_interval)

    async def ws_loop(self) -> None:
        reconnect_delay = self.cfg.ws_reconnect_delay_ms / 1000.0
        hb_interval = self.cfg.ws_heartbeat_interval_ms / 1000.0
        send_interval = self.cfg.send_interval_ms / 1000.0

        while True:
            try:
                async with websockets.connect(
                    self.cfg.ws_uri,
                    ping_interval=20,
                    ping_timeout=20,
                    close_timeout=5,
                    max_size=2 * 1024 * 1024,
                ) as ws:
                    await self.reset_ws_state_on_connect()
                    await self.send_register(ws)

                    recv_task = asyncio.create_task(self.ws_recv_loop(ws), name="ws_recv")
                    next_send = time.monotonic()
                    next_hb = time.monotonic() + hb_interval

                    while True:
                        now = time.monotonic()
                        if now >= next_hb:
                            await self.ws_send_json(ws, {"type": "heartbeat"})
                            next_hb = now + hb_interval

                        if now >= next_send:
                            await self.try_send_servo_delta(ws, now)
                            next_send = now + send_interval

                        if recv_task.done():
                            recv_task.result()
                            break

                        await asyncio.sleep(0.005)
            except ConnectionClosed as exc:
                logging.warning("WebSocket断开: code=%s, reason=%s", exc.code, exc.reason)
            except Exception as exc:
                logging.warning("WebSocket异常: %s", exc)

            await asyncio.sleep(reconnect_delay)

    async def reset_ws_state_on_connect(self) -> None:
        async with self.state_lock:
            self.ws_target_id = ""
            self.need_full_sync = True
            self.last_sent_valid = [False] * SERVO_COUNT
        logging.info("WebSocket连接成功，等待绑定目标")

    async def send_register(self, ws: websockets.WebSocketClientProtocol) -> None:
        msg = {"type": "register", "name": self.cfg.ws_client_name}
        await self.ws_send_json(ws, msg)

    async def ws_recv_loop(self, ws: websockets.WebSocketClientProtocol) -> None:
        async for text in ws:
            await self.handle_ws_message(ws, text)

    async def handle_ws_message(self, ws: websockets.WebSocketClientProtocol, text: str) -> None:
        try:
            obj = json.loads(text)
        except json.JSONDecodeError:
            logging.warning("收到非法JSON: %s", text)
            return

        msg_type = obj.get("type", "")
        if msg_type == "heartbeat":
            await self.ws_send_json(ws, {"type": "heartbeat"})
            return

        if msg_type == "connected":
            client_id = obj.get("clientId", "")
            if client_id:
                async with self.state_lock:
                    self.ws_self_id = client_id
                logging.info("记录本机clientId=%s", client_id)

            await self.try_pick_target_id_from_list(obj.get("onlineClients"))
            await self.try_pick_target_id_from_list(obj.get("onlineUsers"))
            return

        if msg_type == "presence":
            await self.try_pick_target_id_from_list(obj.get("onlineClients"))
            await self.try_pick_target_id_from_list(obj.get("onlineUsers"))
            return

    async def try_pick_target_id_from_list(self, users: Any) -> None:
        if not isinstance(users, list):
            return

        async with self.state_lock:
            self_id = self.ws_self_id
            preferred_name = self.cfg.ws_preferred_target_name.strip()

            candidates: list[tuple[str, str]] = []
            for item in users:
                if not isinstance(item, dict):
                    continue
                uid = str(item.get("id", "")).strip()
                name = str(item.get("name", "")).strip()
                if not uid:
                    continue
                if self_id and uid == self_id:
                    continue
                candidates.append((uid, name))

            if not candidates:
                return

            picked: Optional[tuple[str, str]] = None
            if preferred_name:
                for uid, name in candidates:
                    if name == preferred_name:
                        picked = (uid, name)
                        break

            if picked is None:
                picked = candidates[0]

            if self.ws_target_id != picked[0]:
                self.ws_target_id = picked[0]
                self.need_full_sync = True
                self.last_sent_valid = [False] * SERVO_COUNT
                logging.info("绑定目标: name=%s id=%s", picked[1], picked[0])

    async def try_send_servo_delta(self, ws: websockets.WebSocketClientProtocol, now: float) -> None:
        async with self.state_lock:
            target_id = self.ws_target_id
            angles = self.latest_angles[:] if self.latest_angles else None
            need_full_sync = self.need_full_sync

        if not target_id:
            if now - self.last_wait_bind_log >= 3.0:
                self.last_wait_bind_log = now
                logging.info("等待绑定: 尚未获取目标ID，跳过发送")
            return

        if not angles:
            return

        delta_items: list[tuple[int, int]] = []
        for idx, angle in enumerate(angles):
            delta = abs(angle - self.last_sent_angles[idx])
            if need_full_sync or (not self.last_sent_valid[idx]) or delta >= self.cfg.send_min_delta_deg:
                delta_items.append((SERVO_ID_MIN + idx, angle))

        if not delta_items:
            if now - self.last_no_change_log >= 5.0:
                self.last_no_change_log = now
                logging.info("舵机无变化，本周期不发送")
            return

        max_count = self.cfg.send_batch_max * self.cfg.send_max_batch_per_tick
        send_items = delta_items[:max_count]
        if len(send_items) < len(delta_items) and now - self.last_send_throttle_log >= 3.0:
            self.last_send_throttle_log = now
            logging.warning("发送限流生效, 本周期发送=%d, 待发送=%d", len(send_items), len(delta_items) - len(send_items))

        content = json.dumps(
            [{"c": sid, "p": pos} for sid, pos in send_items],
            separators=(",", ":"),
            ensure_ascii=False,
        )
        payload = {"type": "servo_control", "to": target_id, "content": content}
        await self.ws_send_json(ws, payload)

        async with self.state_lock:
            for sid, pos in send_items:
                idx = sid - SERVO_ID_MIN
                if 0 <= idx < SERVO_COUNT:
                    self.last_sent_angles[idx] = pos
                    self.last_sent_valid[idx] = True

            if self.need_full_sync and len(send_items) == len(delta_items):
                self.need_full_sync = False
                logging.info("全量同步完成，切换增量发送")

    async def ws_send_json(self, ws: websockets.WebSocketClientProtocol, obj: dict[str, Any]) -> None:
        await ws.send(json.dumps(obj, separators=(",", ":"), ensure_ascii=False))


def clamp_int(v: int, lo: int, hi: int) -> int:
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_adc_frame(frame: bytes) -> Optional[tuple[int, int, list[float]]]:
    if len(frame) != FRAME_SIZE:
        logging.warning("I2C帧长度不匹配: len=%d expect=%d", len(frame), FRAME_SIZE)
        return None

    if frame[0:2] != FRAME_MAGIC:
        logging.warning("I2C帧magic错误")
        return None

    version = frame[2]
    channel_count = frame[3]
    if version != FRAME_VERSION or channel_count != SERVO_COUNT:
        logging.warning("I2C帧版本或通道数错误: ver=%d cnt=%d", version, channel_count)
        return None

    payload = frame[:-2]
    recv_crc = int.from_bytes(frame[-2:], "little")
    calc_crc = crc16_ccitt_false(payload)
    if recv_crc != calc_crc:
        logging.warning("I2C帧CRC错误: recv=0x%04x calc=0x%04x", recv_crc, calc_crc)
        return None

    seq = int.from_bytes(frame[4:6], "little")
    uptime_ms = int.from_bytes(frame[6:10], "little")
    servo_id_min = frame[10]
    if servo_id_min != SERVO_ID_MIN:
        logging.warning("I2C帧servo_id_min不匹配: %d", servo_id_min)
        return None

    angles: list[float] = []
    base = FRAME_HEADER_SIZE
    for idx in range(SERVO_COUNT):
        pos = base + idx * 2
        angle_x10 = int.from_bytes(frame[pos : pos + 2], "little")
        angles.append(angle_x10 / 10.0)

    return seq, uptime_ms, angles


def parse_i2c_addr(value: Any) -> int:
    if isinstance(value, int):
        addr = value
    elif isinstance(value, str):
        text = value.strip().lower()
        if text.startswith("0x"):
            addr = int(text, 16)
        else:
            addr = int(text, 10)
    else:
        raise ValueError(f"不支持的i2c_addr类型: {type(value)}")

    if not (0x08 <= addr <= 0x77):
        raise ValueError(f"i2c_addr超出7-bit地址范围: {addr} (0x{addr:02x})")
    return addr


def load_config(path: Optional[str]) -> BridgeConfig:
    cfg = BridgeConfig()
    if not path:
        return cfg

    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)

    allowed = set(asdict(cfg).keys())
    for k, v in raw.items():
        if k in allowed:
            if k == "i2c_addr":
                setattr(cfg, k, parse_i2c_addr(v))
            else:
                setattr(cfg, k, v)
    return cfg


async def async_main(args: argparse.Namespace) -> None:
    cfg = load_config(args.config)
    if args.ws_uri:
        cfg.ws_uri = args.ws_uri
    if args.client_name:
        cfg.ws_client_name = args.client_name

    logging.info(
        "启动桥接: ws=%s i2c_bus=%d i2c_addr=%d(0x%02x)",
        cfg.ws_uri,
        cfg.i2c_bus,
        cfg.i2c_addr,
        cfg.i2c_addr,
    )
    bridge = RaspiWsBridge(cfg)
    await bridge.run()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RasPi I2C<->WebSocket bridge")
    parser.add_argument("--config", default=None, help="配置文件路径(JSON)")
    parser.add_argument("--ws-uri", default=None, help="覆盖WebSocket地址")
    parser.add_argument("--client-name", default=None, help="覆盖注册名称")
    parser.add_argument("--log-level", default="INFO", help="日志级别, 如 INFO/DEBUG")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(message)s",
    )

    try:
        asyncio.run(async_main(args))
    except KeyboardInterrupt:
        logging.info("收到退出信号，桥接停止")


if __name__ == "__main__":
    main()
