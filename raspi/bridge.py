#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import time
import errno
from dataclasses import asdict, dataclass, field
from typing import Any, Optional

try:
    from smbus2 import SMBus, i2c_msg
except ImportError as exc:  # pragma: no cover
    raise SystemExit("缺少依赖 smbus2，请先在 raspi 目录执行: uv sync --frozen") from exc

try:
    import websockets
    from websockets.exceptions import ConnectionClosed
except ImportError as exc:  # pragma: no cover
    raise SystemExit("缺少依赖 websockets，请先在 raspi 目录执行: uv sync --frozen") from exc

SERVO_ID_MIN = 21
SERVO_ID_MAX = 37
SERVO_COUNT = SERVO_ID_MAX - SERVO_ID_MIN + 1
SEND_MIN_DEG = -90
SEND_MAX_DEG = 90

FRAME_MAGIC = b"\xAA\x55"
FRAME_VERSION = 2
FRAME_SIZE = 48
FRAME_HEADER_SIZE = 12


@dataclass
class BridgeConfig:
    i2c_bus: int = 1
    i2c_addr: int = 0x28
    i2c_poll_interval_ms: int = 20
    i2c_retry_delay_ms: int = 200

    ws_uri: str = "ws://192.168.0.100:9102/"
    ws_client_name: str = "rpi-body"
    ws_preferred_target_name: str = ""
    ws_character_name: str = "jiyuan"
    ws_is_bus_servo: bool = True
    ws_servo_speed: int = 100
    ws_reconnect_delay_ms: int = 2000
    ws_heartbeat_interval_ms: int = 15000

    send_interval_ms: int = 30
    send_min_delta_deg: int = 4
    send_batch_max: int = 8
    send_max_batch_per_tick: int = 3

    filter_alpha: float = 0.16
    filter_deadzone_deg: int = 10
    filter_max_step_per_tick: int = 4
    filter_confirm_frames: int = 2

    # 舵机方向修正：当某些舵机安装/定义方向相反时，对该 id 的角度做取反。
    # 例如采样得到 -90，但实际需要 +90，则将对应 servo_id 设置为 true。
    servo_reverse_map: dict[int, bool] = field(default_factory=dict)


class ServoOutputFilter:
    def __init__(
        self,
        alpha: float,
        deadzone: int,
        max_step: int,
        confirm_frames: int,
        min_value: int,
        max_value: int,
        count: int,
    ) -> None:
        self.alpha = alpha
        self.deadzone = deadzone
        self.max_step = max_step
        self.confirm_frames = max(1, confirm_frames)
        self.min_value = min_value
        self.max_value = max_value
        self.filtered = [0.0] * count
        self.stable = [0] * count
        self.inited = [False] * count
        self.pending_dir = [0] * count
        self.pending_count = [0] * count

    def apply(self, idx: int, raw_angle: float) -> int:
        if not self.inited[idx]:
            out = clamp_int(int(round(raw_angle)), self.min_value, self.max_value)
            self.filtered[idx] = raw_angle
            self.stable[idx] = out
            self.inited[idx] = True
            return out

        self.filtered[idx] += self.alpha * (raw_angle - self.filtered[idx])
        candidate = clamp_int(int(round(self.filtered[idx])), self.min_value, self.max_value)
        delta = candidate - self.stable[idx]
        abs_delta = abs(delta)

        if abs_delta <= self.deadzone:
            self.pending_dir[idx] = 0
            self.pending_count[idx] = 0
            return self.stable[idx]

        # 小幅变化要求连续多帧同向确认，降低ADC毛刺导致的舵机抖动
        if self.confirm_frames > 1 and abs_delta < self.deadzone * 3:
            direction = 1 if delta > 0 else -1
            if self.pending_dir[idx] == direction:
                self.pending_count[idx] += 1
            else:
                self.pending_dir[idx] = direction
                self.pending_count[idx] = 1

            if self.pending_count[idx] < self.confirm_frames:
                return self.stable[idx]

            self.pending_dir[idx] = 0
            self.pending_count[idx] = 0

        if delta > self.max_step:
            delta = self.max_step
        elif delta < -self.max_step:
            delta = -self.max_step

        self.stable[idx] = clamp_int(self.stable[idx] + delta, self.min_value, self.max_value)
        return self.stable[idx]


class Esp32I2CClient:
    def __init__(self, bus_id: int, slave_addr: int) -> None:
        self.bus = SMBus(bus_id)
        self.addr = slave_addr

    def read_frame(self) -> bytes:
        # 新版协议里不再需要先 write 命令再 read。
        # 主机对从机地址直接发起固定长度 read；ESP 侧会在 master request 事件发生时回填一整帧。
        msg = i2c_msg.read(self.addr, FRAME_SIZE)
        self.bus.i2c_rdwr(msg)
        return bytes(msg)

    def close(self) -> None:
        self.bus.close()


class RaspiWsBridge:
    def __init__(self, cfg: BridgeConfig) -> None:
        self.cfg = cfg
        self.i2c_client = Esp32I2CClient(cfg.i2c_bus, cfg.i2c_addr)
        self.servo_filter = ServoOutputFilter(
            alpha=cfg.filter_alpha,
            deadzone=cfg.filter_deadzone_deg,
            max_step=cfg.filter_max_step_per_tick,
            confirm_frames=cfg.filter_confirm_frames,
            min_value=SEND_MIN_DEG,
            max_value=SEND_MAX_DEG,
            count=SERVO_COUNT,
        )

        self.servo_reverse_mask = build_servo_reverse_mask(cfg.servo_reverse_map)
        self.any_servo_reversed = any(self.servo_reverse_mask)
        if self.any_servo_reversed:
            reversed_ids = [
                str(SERVO_ID_MIN + idx) for idx, reversed_ in enumerate(self.servo_reverse_mask) if reversed_
            ]
            logging.info("启用servo反向映射: ids=%s", ",".join(reversed_ids))

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
                parsed_frame = parse_adc_frame(frame)
                if parsed_frame is None:
                    await asyncio.sleep(retry_interval)
                    continue

                seq, uptime_ms, send_angles = parsed_frame

                # 方向修正应尽早应用，保证后续滤波、死区、增量判断在同一坐标系下工作。
                if self.any_servo_reversed:
                    send_angles = apply_servo_reverse_mask(send_angles, self.servo_reverse_mask)
                filtered = [self.servo_filter.apply(i, send_angles[i]) for i in range(SERVO_COUNT)]

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
            self.last_sent_angles = [0] * SERVO_COUNT
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
            latest_seq = self.latest_seq
            latest_uptime_ms = self.latest_uptime_ms

        if not target_id:
            if now - self.last_wait_bind_log >= 3.0:
                self.last_wait_bind_log = now
                logging.info("等待绑定: 尚未获取目标ID，跳过发送")
            return

        if not angles:
            return

        # 先收集“未同步成功”的通道，避免被前几个频繁变化通道长期挤占带宽
        unsynced_items: list[tuple[int, int]] = []
        changed_items: list[tuple[int, int]] = []
        for idx, angle in enumerate(angles):
            sid = SERVO_ID_MIN + idx
            if need_full_sync and (not self.last_sent_valid[idx]):
                unsynced_items.append((sid, angle))
                continue

            delta = abs(angle - self.last_sent_angles[idx])
            if (not self.last_sent_valid[idx]) or delta >= self.cfg.send_min_delta_deg:
                changed_items.append((sid, angle))

        delta_items = unsynced_items + changed_items

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

        send_positions = [
            (sid, clamp_int(int(pos), SEND_MIN_DEG, SEND_MAX_DEG)) for sid, pos in send_items
        ]
        send_mode = "full_sync" if need_full_sync else "delta"
        for log_line in build_servo_send_log_lines(
            latest_seq,
            latest_uptime_ms,
            send_positions,
            send_mode=send_mode,
        ):
            logging.info("%s", log_line)

        speed = clamp_int(int(self.cfg.ws_servo_speed), 0, 1000)
        content = json.dumps(
            [
                {
                    "character_name": self.cfg.ws_character_name,
                    "web_servo": {
                        "is_bus_servo": bool(self.cfg.ws_is_bus_servo),
                        "servo_id": sid,
                        "position": pos,
                        "speed": speed,
                    },
                }
                for sid, pos in send_positions
            ],
            separators=(",", ":"),
            ensure_ascii=False,
        )
        payload = {"type": "servo_control", "to": target_id, "content": content}
        await self.ws_send_json(ws, payload)

        async with self.state_lock:
            for sid, pos in send_positions:
                idx = sid - SERVO_ID_MIN
                if 0 <= idx < SERVO_COUNT:
                    self.last_sent_angles[idx] = pos
                    self.last_sent_valid[idx] = True

            if self.need_full_sync and all(self.last_sent_valid):
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


def clamp_float(v: float, lo: float, hi: float) -> float:
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def format_signed_angle_deg(angle_deg: int) -> str:
    if angle_deg > 0:
        return f"+{angle_deg}"
    return str(angle_deg)


def build_servo_send_log_lines(
    seq: int,
    uptime_ms: int,
    send_items: list[tuple[int, int]],
    *,
    send_mode: str,
    chunk_size: int = 8,
) -> list[str]:
    if chunk_size <= 0:
        raise ValueError("chunk_size must be positive")
    if not send_items:
        return []

    total_chunks = (len(send_items) + chunk_size - 1) // chunk_size
    log_lines: list[str] = []
    for chunk_index, start in enumerate(range(0, len(send_items), chunk_size), start=1):
        chunk = send_items[start : start + chunk_size]
        chunk_text = " ".join(
            f"{servo_id}:{format_signed_angle_deg(angle_deg)}" for servo_id, angle_deg in chunk
        )
        log_lines.append(
            "发送舵机角度: "
            f"mode={send_mode} seq={seq} uptime_ms={uptime_ms} "
            f"chunk={chunk_index}/{total_chunks} {chunk_text}"
        )
    return log_lines


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


def _parse_adc_frame(frame: bytes, *, log_errors: bool) -> Optional[tuple[int, int, list[float]]]:
    if len(frame) != FRAME_SIZE:
        if log_errors:
            logging.warning("I2C帧长度不匹配: len=%d expect=%d", len(frame), FRAME_SIZE)
        return None

    frame_head_hex = frame[:8].hex(" ")

    if frame[0:2] != FRAME_MAGIC:
        if log_errors:
            logging.warning("I2C帧magic错误: head=%s", frame_head_hex)
        return None

    version = frame[2]
    channel_count = frame[3]
    if version != FRAME_VERSION or channel_count != SERVO_COUNT:
        if log_errors:
            logging.warning(
                "I2C帧版本或通道数错误: ver=%d cnt=%d expect_ver=%d expect_cnt=%d head=%s",
                version,
                channel_count,
                FRAME_VERSION,
                SERVO_COUNT,
                frame_head_hex,
            )
        return None

    payload = frame[:-2]
    recv_crc = int.from_bytes(frame[-2:], "little")
    calc_crc = crc16_ccitt_false(payload)
    if recv_crc != calc_crc:
        if log_errors:
            logging.warning(
                "I2C帧CRC错误: recv=0x%04x calc=0x%04x head=%s",
                recv_crc,
                calc_crc,
                frame_head_hex,
            )
        return None

    seq = int.from_bytes(frame[4:6], "little")
    uptime_ms = int.from_bytes(frame[6:10], "little")
    servo_id_min = frame[10]
    if servo_id_min != SERVO_ID_MIN:
        if log_errors:
            logging.warning(
                "I2C帧servo_id_min不匹配: got=%d expect=%d head=%s",
                servo_id_min,
                SERVO_ID_MIN,
                frame_head_hex,
            )
        return None

    send_angles: list[float] = []
    base = FRAME_HEADER_SIZE
    for idx in range(SERVO_COUNT):
        pos = base + idx * 2
        send_x10 = int.from_bytes(frame[pos : pos + 2], "little", signed=True)
        send_angles.append(clamp_float(send_x10 / 10.0, float(SEND_MIN_DEG), float(SEND_MAX_DEG)))

    return seq, uptime_ms, send_angles


def parse_adc_frame(frame: bytes) -> Optional[tuple[int, int, list[float]]]:
    return _parse_adc_frame(frame, log_errors=True)


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


def parse_servo_reverse_map(value: Any) -> dict[int, bool]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError("servo_reverse_map必须是JSON对象(dict)，如 {\"21\": true}")

    out: dict[int, bool] = {}
    for raw_sid, raw_reverse in value.items():
        sid: Optional[int] = None
        if isinstance(raw_sid, int):
            sid = raw_sid
        elif isinstance(raw_sid, str):
            text = raw_sid.strip()
            if text:
                try:
                    sid = int(text, 10)
                except ValueError:
                    sid = None

        if sid is None:
            logging.warning("servo_reverse_map忽略非法servo_id键: %r", raw_sid)
            continue

        if not (SERVO_ID_MIN <= sid <= SERVO_ID_MAX):
            logging.warning(
                "servo_reverse_map忽略越界servo_id: %d (允许范围 %d~%d)",
                sid,
                SERVO_ID_MIN,
                SERVO_ID_MAX,
            )
            continue

        reverse: Optional[bool] = None
        if isinstance(raw_reverse, bool):
            reverse = raw_reverse
        elif isinstance(raw_reverse, int) and raw_reverse in (0, 1):
            reverse = bool(raw_reverse)
        elif isinstance(raw_reverse, str):
            t = raw_reverse.strip().lower()
            if t in ("1", "true", "yes", "y", "on"):
                reverse = True
            elif t in ("0", "false", "no", "n", "off"):
                reverse = False

        if reverse is None:
            logging.warning(
                "servo_reverse_map忽略非法reverse值: servo_id=%d value=%r (支持 true/false 或 0/1)",
                sid,
                raw_reverse,
            )
            continue

        out[sid] = reverse

    return out


def build_servo_reverse_mask(reverse_map: dict[int, bool]) -> list[bool]:
    mask = [False] * SERVO_COUNT
    for sid, reverse in reverse_map.items():
        if not reverse:
            continue
        idx = sid - SERVO_ID_MIN
        if 0 <= idx < SERVO_COUNT:
            mask[idx] = True
    return mask


def apply_servo_reverse_mask(angles: list[float], mask: list[bool]) -> list[float]:
    if len(angles) != len(mask):
        return angles
    return [(-angle if mask[idx] else angle) for idx, angle in enumerate(angles)]


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
            elif k == "servo_reverse_map":
                setattr(cfg, k, parse_servo_reverse_map(v))
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
