"""Single 50 Hz ONNX inference and Unitree LowCmd loop."""

from __future__ import annotations

from datetime import datetime
from hashlib import sha256
import json
from pathlib import Path
from threading import Event, Lock, Thread
from time import monotonic, sleep

import numpy as np
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__LowCmd_,
    unitree_hg_msg_dds__LowState_,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as LowCmdHG
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowStateHG
from unitree_sdk2py.utils.crc import CRC

from .config import ControllerConfig, POLICY_JOINT_COUNT
from .imu import get_gravity_orientation, transform_torso_imu
from .policy import OBSERVATION_SIZE, PolicyBank
from .state_machine import (
    MODEL_ACCURATE_ARRIVAL,
    MODEL_ARM_STAND,
    MODEL_ARM_WALK,
    SEMANTICS_TARGET_POSE,
    ControlSelection,
    LocomotionStateMachine,
)
from .totarget_logger import ToTargetLogger


ARM_JOINT_NAMES = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

OBS_ANGULAR_VELOCITY = slice(0, 3)
OBS_GRAVITY = slice(3, 6)
OBS_COMMAND = slice(6, 9)
OBS_JOINT_POSITION = slice(9, 38)
OBS_JOINT_VELOCITY = slice(38, 67)
OBS_PREVIOUS_ACTION = slice(67, 96)
MOTION_SERVICE_RETRY_CODES = {3102, 3104}


class UnitreeController:
    """Own the only LowCmd publisher and the only inference thread."""

    def __init__(
        self,
        config: ControllerConfig,
        policies: PolicyBank,
        state_machine: LocomotionStateMachine,
        totarget_log_directory: Path,
        totarget_model_path: Path,
    ) -> None:
        self._config = config
        self._policies = policies
        self._state_machine = state_machine
        self._stop = Event()
        self._first_frame = Event()
        self._thread: Thread | None = None
        self._control_error: str | None = None
        self._has_taken_control = False
        self._command_lock = Lock()
        self._crc = CRC()
        self._totarget_logger = ToTargetLogger(totarget_log_directory)
        self._totarget_metadata = {
            "model": MODEL_ACCURATE_ARRIVAL,
            "model_path": str(totarget_model_path),
            "model_sha256": _sha256_file(totarget_model_path),
            "control_frequency_hz": 1.0 / config.control_dt,
            "control_dt_s": config.control_dt,
            "imu_type": config.imu_type,
            "policy_joint_names": list(config.policy_joint_names),
            "motor_joint_names": list(config.motor_joint_names),
            "motor_indices": list(config.motor_indices),
            "observation_layout": {
                "angular_velocity": [0, 3],
                "gravity": [3, 6],
                "command": [6, 9],
                "joint_position": [9, 38],
                "joint_velocity": [38, 67],
                "previous_action": [67, 96],
            },
            "default_angles": list(config.default_angles),
            "kps": list(config.kps),
            "kds": list(config.kds),
            "angular_velocity_scale": config.angular_velocity_scale,
            "joint_position_scale": config.joint_position_scale,
            "joint_velocity_scale": config.joint_velocity_scale,
            "action_scale": config.action_scale,
            "lowcmd_contract": {
                "motor_mode": 1,
                "mode_pr": 0,
                "tau_feedforward": 0.0,
            },
        }

        self._motor_to_policy = np.asarray(
            [
                config.motor_joint_names.index(name)
                for name in config.policy_joint_names
            ],
            dtype=np.int64,
        )
        self._policy_to_motor = np.asarray(
            [
                config.policy_joint_names.index(name)
                for name in config.motor_joint_names
            ],
            dtype=np.int64,
        )
        self._arm_indices = np.asarray(
            [config.policy_joint_names.index(name) for name in ARM_JOINT_NAMES],
            dtype=np.int64,
        )
        self._default_angles = np.asarray(config.default_angles, dtype=np.float32)
        self._kps = np.asarray(config.kps, dtype=np.float32)
        self._kds = np.asarray(config.kds, dtype=np.float32)
        self._arm_stand_default_angles = np.asarray(
            config.arm_stand_default_angles,
            dtype=np.float32,
        )
        self._arm_stand_kps = np.asarray(config.arm_stand_kps, dtype=np.float32)
        self._arm_stand_kds = np.asarray(config.arm_stand_kds, dtype=np.float32)
        self._max_velocity_command = np.asarray(
            config.max_velocity_command,
            dtype=np.float32,
        )

        self._observation = np.zeros(OBSERVATION_SIZE, dtype=np.float32)
        self._previous_action = np.zeros(POLICY_JOINT_COUNT, dtype=np.float32)
        self._last_target = np.zeros(POLICY_JOINT_COUNT, dtype=np.float32)
        self._active_model_name: str | None = None
        self._inference_frame_index = 0
        self._switch_started_at = 0.0
        self._switch_from_target = np.zeros(
            POLICY_JOINT_COUNT,
            dtype=np.float32,
        )
        self._arm_baseline = np.zeros(len(ARM_JOINT_NAMES), dtype=np.float32)

        self._low_command = unitree_hg_msg_dds__LowCmd_()
        self._low_state = unitree_hg_msg_dds__LowState_()
        self._last_lowstate_received_at = 0.0
        self._publisher = ChannelPublisher(config.lowcmd_topic, LowCmdHG)
        self._publisher.Init()
        self._subscriber = ChannelSubscriber(config.lowstate_topic, LowStateHG)
        self._subscriber.Init(self._receive_low_state, 10)

        self._wait_for_low_state()
        self._initialize_low_command()
        self._last_target = self._current_policy_positions()

    @property
    def control_error(self) -> str | None:
        return self._control_error

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def whole_body_positions(self) -> tuple[float, ...]:
        """Return measured joint angles in the public motor joint order."""

        low_state = self._low_state
        return tuple(
            float(low_state.motor_state[index].q)
            for index in self._config.motor_indices
        )

    def take_control_and_start(self) -> None:
        enter_debug_mode(self._config.motion_release_timeout_s)
        self._has_taken_control = True
        self._move_to_default_position()
        self._stop.clear()
        self._thread = Thread(
            target=self._run,
            name="unitree-control-50hz",
            daemon=True,
        )
        self._thread.start()
        if not self._first_frame.wait(self._config.first_frame_timeout_s):
            raise TimeoutError("the 50 Hz controller did not produce its first frame")
        if self._control_error is not None:
            raise RuntimeError(self._control_error)

    def confirm_healthy_for(self, duration_s: float) -> None:
        """Confirm continuous free-walk zero control before initialization."""

        if duration_s <= 0.0:
            raise ValueError("initialization stand duration must be positive")
        deadline = monotonic() + duration_s
        while monotonic() < deadline:
            if self._control_error is not None:
                raise RuntimeError(self._control_error)
            if not self.is_running:
                raise RuntimeError("50 Hz control thread stopped during initialization")
            remaining = deadline - monotonic()
            if self._stop.wait(min(self._config.control_dt, remaining)):
                raise RuntimeError(
                    "50 Hz control thread was stopped during initialization"
                )

    def close(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(self._config.stop_timeout_s)
            if thread.is_alive():
                raise RuntimeError("the 50 Hz control thread did not stop")
        self._thread = None
        log_error: Exception | None = None
        try:
            self._totarget_logger.close()
        except Exception as error:
            log_error = error
        finally:
            if self._has_taken_control:
                self._send_damping(self._config.fault_damping_duration_s)
        if log_error is not None:
            raise log_error

    def _receive_low_state(self, message: LowStateHG) -> None:
        self._low_state = message
        self._last_lowstate_received_at = monotonic()

    def _wait_for_low_state(self) -> None:
        deadline = monotonic() + self._config.lowstate_startup_timeout_s
        while int(self._low_state.tick) == 0:
            if monotonic() >= deadline:
                raise TimeoutError(
                    f"no valid LowState on {self._config.lowstate_topic}"
                )
            sleep(self._config.control_dt)

    def _initialize_low_command(self) -> None:
        self._low_command.mode_machine = int(self._low_state.mode_machine)
        self._low_command.mode_pr = 0
        for motor in self._low_command.motor_cmd:
            motor.mode = 1
            motor.q = 0.0
            motor.qd = 0.0
            motor.kp = 0.0
            motor.kd = 0.0
            motor.tau = 0.0

    def _move_to_default_position(self) -> None:
        start_motor = np.asarray(
            [
                self._low_state.motor_state[index].q
                for index in self._config.motor_indices
            ],
            dtype=np.float32,
        )
        target_motor = self._default_angles[self._policy_to_motor]
        kp_motor = self._kps[self._policy_to_motor]
        kd_motor = self._kds[self._policy_to_motor]
        step_count = int(
            round(self._config.startup_move_s / self._config.control_dt)
        )
        for step in range(step_count):
            if (
                monotonic() - self._last_lowstate_received_at
                > self._config.lowstate_runtime_timeout_s
            ):
                raise RuntimeError("LowState stream timed out during startup motion")
            alpha = (step + 1) / step_count
            blend = alpha**3 * (10.0 + alpha * (-15.0 + 6.0 * alpha))
            target = start_motor + (target_motor - start_motor) * blend
            self._write_targets(
                target,
                kp_motor,
                kd_motor,
                np.zeros(POLICY_JOINT_COUNT, dtype=np.float32),
            )
            self._send_command()
            sleep(self._config.control_dt)
        self._last_target = self._default_angles.copy()

    def _run(self) -> None:
        deadline = monotonic()
        try:
            while not self._stop.is_set():
                now = monotonic()
                self._run_frame(now)
                self._first_frame.set()
                deadline += self._config.control_dt
                remaining = deadline - monotonic()
                if remaining > 0.0:
                    self._stop.wait(remaining)
                else:
                    deadline = monotonic()
        except Exception as error:
            self._control_error = f"50 Hz control loop failed: {error}"
            self._first_frame.set()
            try:
                self._send_damping(self._config.fault_damping_duration_s)
            except Exception as damping_error:
                self._control_error += f"; damping failed: {damping_error}"
        finally:
            try:
                self._totarget_logger.end_session("control_loop_stopped")
            except Exception as log_error:
                if self._control_error is None:
                    self._control_error = (
                        f"ToTarget log finalization failed: {log_error}"
                    )
                else:
                    self._control_error += (
                        f"; ToTarget log finalization failed: {log_error}"
                    )

    def _run_frame(self, now: float) -> None:
        if (
            self._last_lowstate_received_at <= 0.0
            or now - self._last_lowstate_received_at
            > self._config.lowstate_runtime_timeout_s
        ):
            raise RuntimeError("LowState stream timed out")

        selection = self._state_machine.select(now)
        if selection.model_name != self._active_model_name:
            if self._active_model_name == MODEL_ACCURATE_ARRIVAL:
                self._totarget_logger.end_session(
                    f"switched_to_{selection.model_name}"
                )
            self._begin_model_switch(selection.model_name, now)
            if selection.model_name == MODEL_ACCURATE_ARRIVAL:
                log_path = self._totarget_logger.start_session(
                    selection.command,
                    self._totarget_metadata,
                )
                print(f"[TOTARGET_LOG] started {log_path}", flush=True)

        frame_wall_time = datetime.now().astimezone()
        inference_frame = self._inference_frame_index
        joint_positions, joint_velocities, quaternion, angular_velocity = (
            self._read_robot_state()
        )
        default_angles, kps, kds = self._parameters_for(selection)
        command = self._command_for(selection)
        self._observation[OBS_ANGULAR_VELOCITY] = (
            angular_velocity * self._config.angular_velocity_scale
        )
        self._observation[OBS_GRAVITY] = get_gravity_orientation(quaternion)
        self._observation[OBS_COMMAND] = command
        self._observation[OBS_JOINT_POSITION] = (
            joint_positions - default_angles
        ) * self._config.joint_position_scale
        self._observation[OBS_JOINT_VELOCITY] = (
            joint_velocities * self._config.joint_velocity_scale
        )
        self._observation[OBS_PREVIOUS_ACTION] = self._previous_action

        inference_started_at = monotonic()
        action = self._policies.get_policy(selection.model_name).infer(
            self._observation
        )
        inference_duration_s = monotonic() - inference_started_at
        raw_model_output = action.copy()
        self._log_inference_frame(selection, command, action)
        target_velocity = np.zeros(POLICY_JOINT_COUNT, dtype=np.float32)
        direct_arm_target: np.ndarray | None = None
        if selection.model_name in {MODEL_ARM_STAND, MODEL_ARM_WALK}:
            direct_arm_target = self._last_target[self._arm_indices].copy()
            if selection.arm_command is not None:
                external_positions = np.asarray(
                    selection.arm_command.positions,
                    dtype=np.float32,
                )
                weight = selection.arm_command.weight
                direct_arm_target = (
                    self._arm_baseline * (1.0 - weight)
                    + external_positions * weight
                )
                target_velocity[self._arm_indices] = (
                    np.asarray(selection.arm_command.velocities, dtype=np.float32)
                    * weight
                )
            action[self._arm_indices] = (
                direct_arm_target - default_angles[self._arm_indices]
            ) / self._config.action_scale

        target = default_angles + action * self._config.action_scale
        target = self._blend_model_switch(target, now)
        if direct_arm_target is not None:
            # Arm commands are never temporally interpolated. With no command
            # for this mode, direct_arm_target is the previous frame target.
            target[self._arm_indices] = direct_arm_target
        self._previous_action = self._previous_action_for(
            target,
            default_angles,
        )
        self._last_target = target.copy()

        self._write_targets(
            target[self._policy_to_motor],
            kps[self._policy_to_motor],
            kds[self._policy_to_motor],
            target_velocity[self._policy_to_motor],
        )
        self._send_command()
        if selection.model_name == MODEL_ACCURATE_ARRIVAL:
            self._totarget_logger.write_frame(
                {
                    "wall_time": frame_wall_time.isoformat(
                        timespec="microseconds"
                    ),
                    "monotonic_time_s": now,
                    "inference_frame": inference_frame,
                    "lowstate": {
                        "tick": int(self._low_state.tick),
                        "mode_machine": int(self._low_state.mode_machine),
                        "received_at_monotonic_s": (
                            self._last_lowstate_received_at
                        ),
                        "age_s": now - self._last_lowstate_received_at,
                    },
                    "mode": {
                        "model": selection.model_name,
                        "high": selection.high_mode,
                        "low": selection.low_mode,
                        "standing_transition": (
                            selection.is_standing_transition
                        ),
                    },
                    "navigation_input": {
                        "semantics": selection.command_semantics,
                        "selected": list(selection.command),
                        "model_input": command.tolist(),
                    },
                    "robot_state_policy_order": {
                        "joint_position": joint_positions.tolist(),
                        "joint_velocity": joint_velocities.tolist(),
                        "imu_quaternion": quaternion.tolist(),
                        "imu_angular_velocity": angular_velocity.tolist(),
                    },
                    "observation": self._observation.tolist(),
                    "model_output": raw_model_output.tolist(),
                    "inference_duration_s": inference_duration_s,
                    "command_policy_order": {
                        "target_position": target.tolist(),
                        "target_velocity": target_velocity.tolist(),
                        "kp": kps.tolist(),
                        "kd": kds.tolist(),
                    },
                    "command_motor_order": {
                        "target_position": target[
                            self._policy_to_motor
                        ].tolist(),
                        "target_velocity": target_velocity[
                            self._policy_to_motor
                        ].tolist(),
                        "kp": kps[self._policy_to_motor].tolist(),
                        "kd": kds[self._policy_to_motor].tolist(),
                        "tau_feedforward": [0.0] * POLICY_JOINT_COUNT,
                    },
                }
            )

    def _log_inference_frame(
        self,
        selection: ControlSelection,
        model_command: np.ndarray,
        model_output: np.ndarray,
    ) -> None:
        arm_command = selection.arm_command
        arm_override = (
            None if arm_command is None else arm_command.to_payload()
        )
        payload = {
            "event": "policy_inference",
            "frame": self._inference_frame_index,
            "model": selection.model_name,
            "high_mode": selection.high_mode,
            "low_mode": selection.low_mode,
            "standing_transition": selection.is_standing_transition,
            "navigation_input": {
                "semantics": selection.command_semantics,
                "selected": list(selection.command),
                "model_input": model_command.tolist(),
            },
            "arm_output_override": arm_override,
            "model_output": model_output.tolist(),
        }
        print(
            f"[INFERENCE] {json.dumps(payload, separators=(',', ':'))}",
            flush=True,
        )
        self._inference_frame_index += 1

    def _begin_model_switch(self, model_name: str, now: float) -> None:
        self._active_model_name = model_name
        self._switch_started_at = now
        self._switch_from_target = self._last_target.copy()
        self._arm_baseline = self._last_target[self._arm_indices].copy()
        self._previous_action.fill(0.0)

    def _parameters_for(
        self,
        selection: ControlSelection,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if selection.model_name == MODEL_ARM_STAND:
            return (
                self._arm_stand_default_angles,
                self._arm_stand_kps,
                self._arm_stand_kds,
            )
        if selection.model_name == MODEL_ARM_WALK:
            return (
                self._default_angles,
                self._arm_stand_kps,
                self._arm_stand_kds,
            )
        return self._default_angles, self._kps, self._kds

    def _previous_action_for(
        self,
        target: np.ndarray,
        default_angles: np.ndarray,
    ) -> np.ndarray:
        # previous_action represents the action that was actually executed.
        # For arm modes this intentionally includes the post-inference external
        # arm output override from target.
        return (
            target - default_angles
        ) / self._config.action_scale

    def _command_for(self, selection: ControlSelection) -> np.ndarray:
        target = np.asarray(selection.command, dtype=np.float32)
        if selection.command_semantics == SEMANTICS_TARGET_POSE:
            return target
        # Velocity commands are applied in the same frame without a ramp or
        # acceleration limiter. Keep only the configured magnitude guard.
        return np.clip(
            target,
            -self._max_velocity_command,
            self._max_velocity_command,
        )

    def _blend_model_switch(self, target: np.ndarray, now: float) -> np.ndarray:
        duration = self._config.model_switch_blend_s
        if duration == 0.0:
            return target
        alpha = min((now - self._switch_started_at) / duration, 1.0)
        blended = self._switch_from_target + (
            target - self._switch_from_target
        ) * alpha
        # The model-switch transition is only for legs and waist. Arm targets
        # always take effect in the current frame, regardless of active model.
        blended[self._arm_indices] = target[self._arm_indices]
        return blended

    def _read_robot_state(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        motor_positions = np.asarray(
            [
                self._low_state.motor_state[index].q
                for index in self._config.motor_indices
            ],
            dtype=np.float32,
        )
        motor_velocities = np.asarray(
            [
                self._low_state.motor_state[index].dq
                for index in self._config.motor_indices
            ],
            dtype=np.float32,
        )
        joint_positions = motor_positions[self._motor_to_policy]
        joint_velocities = motor_velocities[self._motor_to_policy]
        quaternion = np.asarray(
            self._low_state.imu_state.quaternion,
            dtype=np.float32,
        )
        angular_velocity = np.asarray(
            self._low_state.imu_state.gyroscope,
            dtype=np.float32,
        )
        if self._config.imu_type == "torso":
            waist_index = self._config.policy_joint_names.index("waist_yaw_joint")
            quaternion, angular_velocity = transform_torso_imu(
                float(joint_positions[waist_index]),
                float(joint_velocities[waist_index]),
                quaternion,
                angular_velocity,
            )
        return (
            joint_positions,
            joint_velocities,
            quaternion,
            angular_velocity,
        )

    def _current_policy_positions(self) -> np.ndarray:
        motor_positions = np.asarray(
            [
                self._low_state.motor_state[index].q
                for index in self._config.motor_indices
            ],
            dtype=np.float32,
        )
        return motor_positions[self._motor_to_policy]

    def _write_targets(
        self,
        target_motor: np.ndarray,
        kp_motor: np.ndarray,
        kd_motor: np.ndarray,
        velocity_motor: np.ndarray,
    ) -> None:
        for array_index, motor_index in enumerate(self._config.motor_indices):
            motor = self._low_command.motor_cmd[motor_index]
            motor.q = float(target_motor[array_index])
            motor.qd = float(velocity_motor[array_index])
            motor.kp = float(kp_motor[array_index])
            motor.kd = float(kd_motor[array_index])
            motor.tau = 0.0

    def _send_command(self) -> None:
        with self._command_lock:
            self._low_command.mode_machine = int(self._low_state.mode_machine)
            self._low_command.crc = self._crc.Crc(self._low_command)
            self._publisher.Write(self._low_command)

    def _send_damping(self, duration_s: float) -> None:
        for motor in self._low_command.motor_cmd:
            motor.q = 0.0
            motor.qd = 0.0
            motor.kp = 0.0
            motor.kd = 8.0
            motor.tau = 0.0
        deadline = monotonic() + duration_s
        while monotonic() < deadline:
            self._send_command()
            sleep(self._config.control_dt)


def _sha256_file(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as model_file:
        for chunk in iter(lambda: model_file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def enter_debug_mode(timeout_s: float) -> None:
    """Release any active high-level mode and confirm low-level debug mode."""

    motion_client = MotionSwitcherClient()
    motion_client.SetTimeout(5.0)
    motion_client.Init()
    deadline = monotonic() + timeout_s

    mode_name = _read_motion_mode(motion_client, deadline)
    if mode_name == "":
        return

    while monotonic() < deadline:
        release_status, _ = motion_client.ReleaseMode()
        if release_status in MOTION_SERVICE_RETRY_CODES:
            sleep(0.2)
            continue
        if release_status != 0:
            raise RuntimeError(
                f"MotionSwitcher ReleaseMode failed: {release_status}"
            )
        break
    else:
        raise TimeoutError("MotionSwitcher ReleaseMode timed out")

    while monotonic() < deadline:
        mode_name = _read_motion_mode(motion_client, deadline)
        if mode_name == "":
            return
        sleep(0.02)
    raise TimeoutError("Unitree did not enter low-level debug mode")


def _read_motion_mode(
    client: MotionSwitcherClient,
    deadline: float,
) -> str:
    while monotonic() < deadline:
        status, result = client.CheckMode()
        if status in MOTION_SERVICE_RETRY_CODES:
            sleep(0.2)
            continue
        if status != 0:
            raise RuntimeError(f"MotionSwitcher CheckMode failed: {status}")
        if (
            not isinstance(result, dict)
            or set(result) != {"name", "form"}
            or not isinstance(result["name"], str)
        ):
            raise RuntimeError(
                f"MotionSwitcher returned an invalid mode result: {result!r}"
            )
        return result["name"]
    raise TimeoutError("MotionSwitcher CheckMode timed out")
