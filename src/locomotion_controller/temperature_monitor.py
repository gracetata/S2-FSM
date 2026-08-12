"""Rate-limited ankle motor temperature monitoring."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from numbers import Real
from typing import Iterable, Sequence


ANKLE_JOINT_NAMES = (
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
)
ANKLE_TEMPERATURE_LIMIT_C = 70.0
ANKLE_TEMPERATURE_WARNING_INTERVAL_S = 5.0


@dataclass(frozen=True)
class MotorTemperatureReading:
    joint_name: str
    motor_index: int
    sensors_c: tuple[float, float]

    @property
    def maximum_c(self) -> float:
        return max(self.sensors_c)


class AnkleTemperatureMonitor:
    """Return hot ankle readings on entry and at a limited reminder rate."""

    def __init__(
        self,
        limit_c: float = ANKLE_TEMPERATURE_LIMIT_C,
        warning_interval_s: float = ANKLE_TEMPERATURE_WARNING_INTERVAL_S,
    ) -> None:
        self._limit_c = float(limit_c)
        self._warning_interval_s = float(warning_interval_s)
        if not isfinite(self._limit_c):
            raise ValueError("temperature limit must be finite")
        if (
            not isfinite(self._warning_interval_s)
            or self._warning_interval_s <= 0
        ):
            raise ValueError(
                "temperature warning interval must be finite and positive"
            )
        self._hot_motor_keys: frozenset[tuple[str, int]] = frozenset()
        self._last_warning_at = 0.0

    @property
    def limit_c(self) -> float:
        return self._limit_c

    def check(
        self,
        samples: Iterable[tuple[str, int, Sequence[Real]]],
        now: float,
    ) -> tuple[MotorTemperatureReading, ...]:
        current_time = float(now)
        hot: list[MotorTemperatureReading] = []
        for joint_name, motor_index, values in samples:
            sensors = tuple(values)
            if len(sensors) != 2:
                raise ValueError(
                    f"{joint_name} motor temperature must contain two values"
                )
            converted = tuple(float(value) for value in sensors)
            if not all(isfinite(value) for value in converted):
                raise ValueError(
                    f"{joint_name} motor temperature must be finite"
                )
            reading = MotorTemperatureReading(
                joint_name=str(joint_name),
                motor_index=int(motor_index),
                sensors_c=(converted[0], converted[1]),
            )
            if reading.maximum_c > self._limit_c:
                hot.append(reading)

        if not hot:
            self._hot_motor_keys = frozenset()
            return ()

        hot_motor_keys = frozenset(
            (reading.joint_name, reading.motor_index) for reading in hot
        )
        newly_hot = hot_motor_keys - self._hot_motor_keys
        should_warn = (
            bool(newly_hot)
            or current_time - self._last_warning_at >= self._warning_interval_s
        )
        self._hot_motor_keys = hot_motor_keys
        if not should_warn:
            return ()
        self._last_warning_at = current_time
        return tuple(hot)
