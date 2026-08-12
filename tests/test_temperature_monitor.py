import unittest

from locomotion_controller.temperature_monitor import (
    ANKLE_JOINT_NAMES,
    AnkleTemperatureMonitor,
)


def samples(*sensor_pairs):
    return tuple(
        (joint_name, motor_index, sensor_pair)
        for motor_index, (joint_name, sensor_pair) in enumerate(
            zip(ANKLE_JOINT_NAMES, sensor_pairs)
        )
    )


class AnkleTemperatureMonitorTest(unittest.TestCase):
    def test_warns_only_above_seventy_for_all_four_ankles(self):
        monitor = AnkleTemperatureMonitor()

        at_limit = monitor.check(
            samples((70, 69), (60, 61), (68, 70), (55, 56)),
            now=1.0,
        )
        self.assertEqual(at_limit, ())

        warning = monitor.check(
            samples((71, 69), (60, 72), (68, 70), (75, 56)),
            now=2.0,
        )
        self.assertEqual(
            tuple(reading.joint_name for reading in warning),
            (
                "left_ankle_pitch_joint",
                "left_ankle_roll_joint",
                "right_ankle_roll_joint",
            ),
        )
        self.assertEqual(warning[0].sensors_c, (71.0, 69.0))

    def test_warning_is_rate_limited_and_rearms_after_cooling(self):
        monitor = AnkleTemperatureMonitor(warning_interval_s=5.0)
        hot = samples((71, 60), (60, 60), (60, 60), (60, 60))
        cool = samples((69, 60), (60, 60), (60, 60), (60, 60))

        self.assertEqual(len(monitor.check(hot, now=10.0)), 1)
        self.assertEqual(monitor.check(hot, now=14.9), ())
        self.assertEqual(len(monitor.check(hot, now=15.0)), 1)
        self.assertEqual(monitor.check(cool, now=15.1), ())
        self.assertEqual(len(monitor.check(hot, now=15.2)), 1)

    def test_newly_hot_ankle_warns_without_waiting_for_interval(self):
        monitor = AnkleTemperatureMonitor(warning_interval_s=5.0)
        first_hot = samples((71, 60), (60, 60), (60, 60), (60, 60))
        second_hot = samples((71, 60), (72, 60), (60, 60), (60, 60))

        self.assertEqual(len(monitor.check(first_hot, now=10.0)), 1)
        warning = monitor.check(second_hot, now=11.0)
        self.assertEqual(len(warning), 2)

    def test_rejects_invalid_sensor_payload(self):
        monitor = AnkleTemperatureMonitor()
        with self.assertRaisesRegex(ValueError, "must contain two values"):
            monitor.check(
                (("left_ankle_pitch_joint", 4, (60,)),),
                now=1.0,
            )


if __name__ == "__main__":
    unittest.main()
