import json
import os
import tempfile
import unittest

import bridge


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
        angles = [-90.0, -45.0, 0.0, 30.0]
        mask = [True, False, True, False]
        out = bridge.apply_servo_reverse_mask(angles, mask)
        self.assertEqual(out, [90.0, -45.0, 0.0, 30.0])
        self.assertEqual(angles, [-90.0, -45.0, 0.0, 30.0])

    def test_load_config_parses_servo_reverse_map(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, "config.json")
            with open(path, "w", encoding="utf-8") as f:
                json.dump({"servo_reverse_map": {"21": True, "22": 0}}, f)

            cfg = bridge.load_config(path)
            self.assertEqual(cfg.servo_reverse_map, {21: True, 22: False})


if __name__ == "__main__":
    unittest.main()

