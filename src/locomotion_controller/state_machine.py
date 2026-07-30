"""Pure four-mode state machine used by the 50 Hz runtime."""

from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from time import monotonic

from .protocol import ArmCommand


HIGH_MODE_NAVIGATION = 1
HIGH_MODE_ARM_STAND = 2
HIGH_MODE_ARM_WALK = 3
HIGH_MODE_STAND_RECOVERY = 4
HIGH_MODES = {
    HIGH_MODE_NAVIGATION,
    HIGH_MODE_ARM_STAND,
    HIGH_MODE_ARM_WALK,
    HIGH_MODE_STAND_RECOVERY,
}

LOW_MODE_VELOCITY = 1
LOW_MODE_TARGET_POSE = 2
LOW_MODES = {LOW_MODE_VELOCITY, LOW_MODE_TARGET_POSE}

MODEL_FREE_WALK = "free_walk"
MODEL_ACCURATE_ARRIVAL = "accurate_arrival"
MODEL_ARM_STAND = "arm_stand"
MODEL_ARM_WALK = "arm_walk"
MODEL_STAND_RECOVERY = "stand_recovery"

SEMANTICS_VELOCITY = "velocity"
SEMANTICS_TARGET_POSE = "target_pose"
ZERO_COMMAND = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ControlSelection:
    model_name: str
    command_semantics: str
    command: tuple[float, float, float]
    arm_command: ArmCommand | None
    high_mode: int | None
    low_mode: int | None
    is_standing_transition: bool


class LocomotionStateMachine:
    """Own mode selection and input routing; never performs inference or I/O."""

    def __init__(
        self,
        stand_duration_s: float,
        navigation_timeout_s: float,
        arm_timeout_s: float,
    ) -> None:
        self._stand_duration_s = float(stand_duration_s)
        self._navigation_timeout_s = float(navigation_timeout_s)
        self._arm_timeout_s = float(arm_timeout_s)
        if self._stand_duration_s <= 0.0:
            raise ValueError("stand duration must be positive")
        if self._navigation_timeout_s <= 0.0 or self._arm_timeout_s <= 0.0:
            raise ValueError("input timeouts must be positive")

        self._lock = Lock()
        self._high_mode: int | None = None
        self._low_mode: int | None = None
        self._navigation_command = ZERO_COMMAND
        self._navigation_received_at = 0.0
        self._arm_command: ArmCommand | None = None
        self._arm_received_at = 0.0
        self._last_arm_sequence: int | None = None
        self._stand_until = 0.0

    def set_high_mode(self, high_mode: int, now: float | None = None) -> bool:
        current_time = monotonic() if now is None else float(now)
        with self._lock:
            if high_mode not in HIGH_MODES:
                self._high_mode = None
                self._low_mode = None
                self._reset_navigation()
                self._reset_arm_input()
                self._stand_until = 0.0
                raise ValueError(
                    f"high-level mode must be one of {sorted(HIGH_MODES)}; "
                    "controller entered free-walk zero standby"
                )
            if high_mode == self._high_mode:
                return False
            self._high_mode = high_mode
            self._reset_navigation()
            if high_mode in {HIGH_MODE_ARM_STAND, HIGH_MODE_ARM_WALK}:
                # A pose received for an earlier mode must not be applied to
                # the newly selected arm policy. The controller holds its
                # previous frame until a post-switch command arrives.
                self._reset_arm_input()
            self._stand_until = (
                current_time + self._stand_duration_s
                if high_mode in {
                    HIGH_MODE_NAVIGATION,
                    HIGH_MODE_ARM_STAND,
                }
                else 0.0
            )
            return True

    def set_low_mode(self, low_mode: int, now: float | None = None) -> bool:
        current_time = monotonic() if now is None else float(now)
        with self._lock:
            if low_mode not in LOW_MODES:
                self._high_mode = None
                self._low_mode = None
                self._reset_navigation()
                self._reset_arm_input()
                self._stand_until = 0.0
                raise ValueError(
                    f"low-level mode must be one of {sorted(LOW_MODES)}; "
                    "controller entered free-walk zero standby"
                )
            if low_mode == self._low_mode:
                return False
            previous_low_mode = self._low_mode
            self._low_mode = low_mode
            self._reset_navigation()
            if (
                self._high_mode == HIGH_MODE_NAVIGATION
                and previous_low_mode == LOW_MODE_VELOCITY
                and low_mode == LOW_MODE_TARGET_POSE
            ):
                self._stand_until = max(
                    self._stand_until,
                    current_time + self._stand_duration_s,
                )
            return True

    def set_navigation_command(
        self,
        command: tuple[float, float, float],
        now: float | None = None,
    ) -> None:
        current_time = monotonic() if now is None else float(now)
        with self._lock:
            self._navigation_command = tuple(float(value) for value in command)
            self._navigation_received_at = current_time

    def set_arm_command(
        self,
        command: ArmCommand,
        now: float | None = None,
    ) -> None:
        current_time = monotonic() if now is None else float(now)
        with self._lock:
            if (
                self._last_arm_sequence is not None
                and command.sequence <= self._last_arm_sequence
            ):
                raise ValueError("arm command sequence must increase")
            self._arm_command = command
            self._arm_received_at = current_time
            self._last_arm_sequence = command.sequence

    def select(self, now: float | None = None) -> ControlSelection:
        current_time = monotonic() if now is None else float(now)
        with self._lock:
            high_mode = self._high_mode
            low_mode = self._low_mode
            navigation_command = self._navigation_command
            navigation_received_at = self._navigation_received_at
            arm_command = self._arm_command
            arm_received_at = self._arm_received_at
            stand_until = self._stand_until

        is_standing = high_mode is None or current_time < stand_until
        if is_standing:
            return ControlSelection(
                model_name=MODEL_FREE_WALK,
                command_semantics=SEMANTICS_VELOCITY,
                command=ZERO_COMMAND,
                arm_command=None,
                high_mode=high_mode,
                low_mode=low_mode,
                is_standing_transition=high_mode is not None,
            )

        is_navigation_fresh = (
            navigation_received_at > 0.0
            and current_time - navigation_received_at <= self._navigation_timeout_s
        )
        is_arm_fresh = (
            arm_command is not None
            and arm_received_at > 0.0
            and current_time - arm_received_at <= self._arm_timeout_s
        )
        fresh_navigation = navigation_command if is_navigation_fresh else ZERO_COMMAND
        fresh_arm = arm_command if is_arm_fresh else None

        if high_mode == HIGH_MODE_NAVIGATION:
            if low_mode == LOW_MODE_TARGET_POSE:
                return ControlSelection(
                    MODEL_ACCURATE_ARRIVAL,
                    SEMANTICS_TARGET_POSE,
                    fresh_navigation,
                    None,
                    high_mode,
                    low_mode,
                    False,
                )
            if low_mode == LOW_MODE_VELOCITY:
                return ControlSelection(
                    MODEL_FREE_WALK,
                    SEMANTICS_VELOCITY,
                    fresh_navigation,
                    None,
                    high_mode,
                    low_mode,
                    False,
                )
            return ControlSelection(
                MODEL_FREE_WALK,
                SEMANTICS_VELOCITY,
                ZERO_COMMAND,
                None,
                high_mode,
                None,
                False,
            )

        if high_mode == HIGH_MODE_ARM_STAND:
            return ControlSelection(
                MODEL_ARM_STAND,
                SEMANTICS_VELOCITY,
                ZERO_COMMAND,
                fresh_arm,
                high_mode,
                low_mode,
                False,
            )

        if high_mode == HIGH_MODE_ARM_WALK:
            # Navigation velocity is the arm-walk model command. The arm
            # command remains a separate post-inference output override.
            velocity_command = (
                fresh_navigation
                if low_mode == LOW_MODE_VELOCITY
                else ZERO_COMMAND
            )
            return ControlSelection(
                MODEL_ARM_WALK,
                SEMANTICS_VELOCITY,
                velocity_command,
                fresh_arm,
                high_mode,
                low_mode,
                False,
            )

        if high_mode == HIGH_MODE_STAND_RECOVERY:
            return ControlSelection(
                MODEL_STAND_RECOVERY,
                SEMANTICS_VELOCITY,
                ZERO_COMMAND,
                None,
                high_mode,
                None,
                False,
            )

        return ControlSelection(
            MODEL_FREE_WALK,
            SEMANTICS_VELOCITY,
            ZERO_COMMAND,
            None,
            None,
            None,
            False,
        )

    def _reset_navigation(self) -> None:
        self._navigation_command = ZERO_COMMAND
        self._navigation_received_at = 0.0

    def _reset_arm_input(self) -> None:
        self._arm_command = None
        self._arm_received_at = 0.0
