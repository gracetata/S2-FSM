"""Single 50 Hz ONNX inference and Unitree LowCmd loop."""

from __future__ import annotations

from threading import Event, Lock, Thread
from time import monotonic, sleep

import numpy as np
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
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
    MODEL_ARM_STAND,
    MODEL_ARM_WALK,
    SEMANTICS_TARGET_POSE,
    ControlSelection,
    LocomotionStateMachine,
)


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
EXPECTED_MOTION_MODE = "ai"
ZERO_TORQUE_FSM_ID = 0
MOTION_SERVICE_RETRY_CODES = {3102, 3104}


class UnitreeController:
    """Own the only LowCmd publisher and the only inference thread."""

    def __init__(
        self,
        config: ControllerConfig,
        policies: PolicyBank,
        state_machine: LocomotionStateMachine,
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
        self._physical_command = np.zeros(3, dtype=np.float32)
        self._last_target = np.zeros(POLICY_JOINT_COUNT, dtype=np.float32)
        self._active_model_name: str | None = None
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

    def take_control_and_start(self) -> None:
        release_motion_mode(self._config.motion_release_timeout_s)
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
        if self._has_taken_control:
            self._send_damping(self._config.fault_damping_duration_s)

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

    def _run_frame(self, now: float) -> None:
        if (
            self._last_lowstate_received_at <= 0.0
            or now - self._last_lowstate_received_at
            > self._config.lowstate_runtime_timeout_s
        ):
            raise RuntimeError("LowState stream timed out")

        selection = self._state_machine.select(now)
        if selection.model_name != self._active_model_name:
            self._begin_model_switch(selection.model_name, now)

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

        action = self._policies.get_policy(selection.model_name).infer(
            self._observation
        )
        target_velocity = np.zeros(POLICY_JOINT_COUNT, dtype=np.float32)
        held_arm_target: np.ndarray | None = None
        if selection.model_name in {MODEL_ARM_STAND, MODEL_ARM_WALK}:
            held_arm_target = self._last_target[self._arm_indices].copy()
            arm_target = held_arm_target
            if selection.arm_command is not None:
                external_positions = np.asarray(
                    selection.arm_command.positions,
                    dtype=np.float32,
                )
                weight = selection.arm_command.weight
                arm_target = (
                    self._arm_baseline * (1.0 - weight)
                    + external_positions * weight
                )
                target_velocity[self._arm_indices] = (
                    np.asarray(selection.arm_command.velocities, dtype=np.float32)
                    * weight
                )
            action[self._arm_indices] = (
                arm_target - default_angles[self._arm_indices]
            ) / self._config.action_scale

        target = default_angles + action * self._config.action_scale
        target = self._blend_model_switch(target, now)
        if held_arm_target is not None and selection.arm_command is None:
            target[self._arm_indices] = held_arm_target
        self._previous_action = (
            target - default_angles
        ) / self._config.action_scale
        self._last_target = target.copy()

        self._write_targets(
            target[self._policy_to_motor],
            kps[self._policy_to_motor],
            kds[self._policy_to_motor],
            target_velocity[self._policy_to_motor],
        )
        self._send_command()

    def _begin_model_switch(self, model_name: str, now: float) -> None:
        self._active_model_name = model_name
        self._switch_started_at = now
        self._switch_from_target = self._last_target.copy()
        self._arm_baseline = self._last_target[self._arm_indices].copy()
        self._previous_action.fill(0.0)
        self._physical_command.fill(0.0)

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
        return self._default_angles, self._kps, self._kds

    def _command_for(self, selection: ControlSelection) -> np.ndarray:
        target = np.asarray(selection.command, dtype=np.float32)
        if selection.command_semantics == SEMANTICS_TARGET_POSE:
            self._physical_command = target
            return target
        target = np.clip(
            target,
            -self._max_velocity_command,
            self._max_velocity_command,
        )
        if selection.is_standing_transition or selection.high_mode is None:
            self._physical_command = target
            return target
        if not self._config.is_command_ramp_enabled:
            self._physical_command = target
            return target
        max_step = np.asarray(
            (
                self._config.command_max_linear_accel
                * self._config.control_dt,
                self._config.command_max_linear_accel
                * self._config.control_dt,
                self._config.command_max_yaw_accel
                * self._config.control_dt,
            ),
            dtype=np.float32,
        )
        delta = np.clip(target - self._physical_command, -max_step, max_step)
        self._physical_command += delta
        return self._physical_command

    def _blend_model_switch(self, target: np.ndarray, now: float) -> np.ndarray:
        duration = self._config.model_switch_blend_s
        if duration == 0.0:
            return target
        alpha = min((now - self._switch_started_at) / duration, 1.0)
        return self._switch_from_target + (
            target - self._switch_from_target
        ) * alpha

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


def release_motion_mode(timeout_s: float) -> None:
    """Verify AI ZeroTorque, then release high-level control for LowCmd."""

    motion_client = MotionSwitcherClient()
    motion_client.SetTimeout(5.0)
    motion_client.Init()
    loco_client = LocoClient()
    loco_client.SetTimeout(5.0)
    loco_client.Init()
    deadline = monotonic() + timeout_s

    mode_name = _read_motion_mode(motion_client, deadline)
    if mode_name != EXPECTED_MOTION_MODE:
        raise RuntimeError(
            "low-level takeover requires MotionSwitcher mode=ai; "
            f"actual mode={mode_name!r}"
        )
    fsm_id = _read_loco_fsm_id(loco_client, deadline)
    if fsm_id != ZERO_TORQUE_FSM_ID:
        raise RuntimeError(
            "low-level takeover requires Loco FSM=0 (ZeroTorque); "
            f"actual FSM={fsm_id}"
        )

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
        if mode_name != EXPECTED_MOTION_MODE:
            raise RuntimeError(
                "MotionSwitcher entered an unexpected mode after release: "
                f"{mode_name!r}"
            )
        sleep(0.02)
    raise TimeoutError("Unitree native motion mode was not released")


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


def _read_loco_fsm_id(client: LocoClient, deadline: float) -> int:
    while monotonic() < deadline:
        status, fsm_id = client.GetFsmId()
        if status in MOTION_SERVICE_RETRY_CODES:
            sleep(0.2)
            continue
        if status != 0:
            raise RuntimeError(f"Loco GetFsmId failed: {status}")
        if not isinstance(fsm_id, int) or isinstance(fsm_id, bool):
            raise RuntimeError(f"Loco returned an invalid FSM ID: {fsm_id!r}")
        return fsm_id
    raise TimeoutError("Loco GetFsmId timed out")
