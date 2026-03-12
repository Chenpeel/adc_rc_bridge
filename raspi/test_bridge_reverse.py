import json
import os
import tempfile
import unittest

import bridge


def build_test_frame(seq: int, uptime_ms: int, send_angles_deg: list[float]) -> bytes:
    frame = bytearray(bridge.FRAME_SIZE)
    frame[0:2] = bridge.FRAME_MAGIC
    frame[2] = bridge.FRAME_VERSION
    frame[3] = bridge.SERVO_COUNT
    frame[4:6] = int(seq).to_bytes(2, "little", signed=False)
    frame[6:10] = int(uptime_ms).to_bytes(4, "little", signed=False)
    frame[10] = bridge.SERVO_ID_MIN
    frame[11] = 0

    for idx in range(bridge.SERVO_COUNT):
        angle = send_angles_deg[idx] if idx < len(send_angles_deg) else 0.0
        send_x10 = int(round(angle * 10.0))
        pos = bridge.FRAME_HEADER_SIZE + idx * 2
        frame[pos : pos + 2] = int(send_x10).to_bytes(2, "little", signed=True)

    crc = bridge.crc16_ccitt_false(bytes(frame[:-2]))
    frame[-2:] = int(crc).to_bytes(2, "little", signed=False)
    return bytes(frame)


class TestServoReverseMap(unittest.TestCase):
    def test_parse_servo_reverse_map_str_keys(self) -> None:
        parsed = bridge.parse_servo_reverse_map({"21": True, "22": False})
        self.assertEqual(parsed, {21: True, 22: False})

    def test_build_servo_reverse_mask(self) -> None:
        mask = bridge.build_servo_reverse_mask(
            {
                bridge.SERVO_ID_MIN: True,
                bridge.SERVO_ID_MIN + 1: False,
                bridge.SERVO_ID_MIN + 2: True,
            }
        )
        self.assertEqual(len(mask), bridge.SERVO_COUNT)
        self.assertTrue(mask[0])
        self.assertFalse(mask[1])
        self.assertTrue(mask[2])

    def test_apply_servo_reverse_mask(self) -> None:
        # reverse 的语义必须是“取反”(angle = -angle)，不是 abs(angle)。
        # 因此：当 angle 为正且 mask=True 时，必须翻转为负数。
        angles = [-90.0, 45.0, 0.0, 30.0]
        mask = [True, True, True, False]
        out = bridge.apply_servo_reverse_mask(angles, mask)
        self.assertEqual(out, [90.0, -45.0, 0.0, 30.0])
        self.assertEqual(angles, [-90.0, 45.0, 0.0, 30.0])

    def test_load_config_parses_servo_reverse_map(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, "config.json")
            with open(path, "w", encoding="utf-8") as f:
                json.dump({"servo_reverse_map": {"21": True, "22": 0}}, f)

            cfg = bridge.load_config(path)
            self.assertEqual(cfg.servo_reverse_map, {21: True, 22: False})

    def test_parse_adc_frame(self) -> None:
        frame = build_test_frame(101, 123556, [84.8, 40.0, 36.6, -1.5])
        parsed = bridge.parse_adc_frame(frame)
        self.assertIsNotNone(parsed)
        assert parsed is not None

        seq, uptime_ms, send_angles = parsed
        self.assertEqual(seq, 101)
        self.assertEqual(uptime_ms, 123556)
        self.assertAlmostEqual(send_angles[0], 84.8)
        self.assertAlmostEqual(send_angles[1], 40.0)
        self.assertAlmostEqual(send_angles[2], 36.6)
        self.assertAlmostEqual(send_angles[3], -1.5)

    def test_build_servo_send_trace_log_lines(self) -> None:
        log_lines = bridge.build_servo_send_trace_log_lines(
            101,
            123556,
            [
                (21, 18.4, -18.4, -18),
                (22, 0.0, 0.0, 0),
                (23, -13.2, 13.2, 13),
                (24, -90.0, 90.0, 90),
            ],
            send_mode="delta",
            chunk_size=2,
        )
        self.assertEqual(
            log_lines,
            [
                (
                    "发送舵机角度: mode=delta seq=101 uptime_ms=123556 "
                    "chunk=1/2 21:frame=+18.4 rev=-18.4 sent=-18 "
                    "22:frame=0.0 rev=0.0 sent=0"
                ),
                (
                    "发送舵机角度: mode=delta seq=101 uptime_ms=123556 "
                    "chunk=2/2 23:frame=-13.2 rev=+13.2 sent=+13 "
                    "24:frame=-90.0 rev=+90.0 sent=+90"
                ),
            ],
        )


if __name__ == "__main__":
    unittest.main()
