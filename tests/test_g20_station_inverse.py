"""The motor->joint decode must invert the station HandCore joint->motor encode.

Station values pinned from linkerhand_retarget config/hand_config.yml (g20). A decode that
disagreed with them reversed the left thumb roll disc and finger side swing (2026-11-04,
docs/reports/2026-11-04-left-thumb-chain).
"""
import unittest

from gamepad_vla.workstation import g20_mapping as g

STATION_COMMAND = {
    "right": ([255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, None, None, None, None, 255, 255, 255, 255, 255],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, None, None, None, None, 0, 0, 0, 0, 0]),
    "left": ([255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 255, None, None, None, None, 255, 255, 255, 255, 255],
             [0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 0, None, None, None, None, 0, 0, 0, 0, 0]),
}


class StationInverseTests(unittest.TestCase):
    def test_command_tables_match_station(self):
        for hand, (lower, upper) in STATION_COMMAND.items():
            self.assertEqual(g.G20_COMMAND_LOWER[hand], lower, hand)
            self.assertEqual(g.G20_COMMAND_UPPER[hand], upper, hand)

    def test_round_trip_through_station_encode(self):
        for hand, (lower, upper) in STATION_COMMAND.items():
            for frac in (0.0, 0.25, 0.6, 1.0):
                motors = [255.0] * g.G20_MOTOR_COUNT
                expected = {}
                for m in range(g.G20_MOTOR_COUNT):
                    if lower[m] is None:
                        continue
                    name = g.G20_JOINT_NAMES[g.G20_URDF_INDEX[hand][m]]
                    lo, hi = g.G20_JOINT_LIMITS[hand][name]
                    q = lo + frac * (hi - lo)
                    expected[name] = q
                    motors[m] = lower[m] + (q - lo) / (hi - lo) * (upper[m] - lower[m])
                decoded = dict(zip(g.G20_JOINT_NAMES, g.motor_positions_to_joint_positions(hand, motors)))
                for name, q in expected.items():
                    self.assertAlmostEqual(decoded[name], q, places=6, msg=f"{hand} {name} frac={frac}")


if __name__ == "__main__":
    unittest.main()
