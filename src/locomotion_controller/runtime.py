"""Control-process backend and its local Unix socket server."""

from __future__ import annotations

import json
import os
from pathlib import Path
import socket
from typing import Any

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from .config import PackageConfig
from .policy import PolicyBank
from .protocol import arm_command_from_payload, parse_navigation_command
from .state_machine import LocomotionStateMachine
from .unitree_controller import UnitreeController


IPC_PROTOCOL = "locomotion_controller.ipc.v1"
MAX_MESSAGE_BYTES = 64 * 1024


class ControlRuntime:
    """Preload policies, start one controller, and accept state updates."""

    def __init__(self, config: PackageConfig) -> None:
        if not config.runtime.is_real_robot_confirmed:
            raise RuntimeError("runtime.confirm_real_robot must be true")
        policies = PolicyBank(config.models)
        state_config = config.state_machine
        self._state_machine = LocomotionStateMachine(
            stand_duration_s=state_config.stand_duration_s,
            navigation_timeout_s=state_config.navigation_timeout_s,
            arm_timeout_s=state_config.arm_timeout_s,
        )
        ChannelFactoryInitialize(0, config.runtime.network_interface)
        self._controller = UnitreeController(
            config.controller,
            policies,
            self._state_machine,
        )
        try:
            self._controller.take_control_and_start()
            self._controller.confirm_healthy_for(
                state_config.initialization_stand_duration_s
            )
        except Exception:
            self._controller.close()
            raise

    def execute(self, request: dict[str, Any]) -> dict[str, object]:
        operation = request["operation"]
        if operation == "status":
            _require_request_keys(request, {"operation"})
            return self._status()
        self._require_healthy()
        if operation == "high_mode":
            _require_request_keys(request, {"operation", "value"})
            changed = self._state_machine.set_high_mode(
                _integer(request["value"], "value")
            )
            return self._mode_response(
                changed,
                "high-level mode accepted",
            )
        if operation == "low_mode":
            _require_request_keys(request, {"operation", "value"})
            changed = self._state_machine.set_low_mode(
                _integer(request["value"], "value")
            )
            return self._mode_response(
                changed,
                "low-level mode accepted",
            )
        if operation == "navigation":
            _require_request_keys(request, {"operation", "values"})
            command = parse_navigation_command(request["values"])
            self._state_machine.set_navigation_command(command)
            return self._response(True, "navigation command accepted")
        if operation == "arm":
            _require_request_keys(request, {"operation", "command"})
            command = arm_command_from_payload(request["command"])
            self._state_machine.set_arm_command(command)
            return self._response(True, "arm command accepted")
        raise ValueError(f"unsupported operation: {operation!r}")

    def close(self) -> None:
        self._controller.close()

    def _status(self) -> dict[str, object]:
        error = self._controller.control_error
        if error is not None:
            return self._response(False, error)
        if not self._controller.is_running:
            return self._response(False, "50 Hz control thread is not running")
        return self._response(
            True,
            "four models ready; initialization zero-velocity stand complete",
        )

    def _require_healthy(self) -> None:
        error = self._controller.control_error
        if error is not None:
            raise RuntimeError(error)
        if not self._controller.is_running:
            raise RuntimeError("50 Hz control thread is not running")

    @staticmethod
    def _response(success: bool, detail: str) -> dict[str, object]:
        return {
            "protocol": IPC_PROTOCOL,
            "success": success,
            "detail": detail,
        }

    @staticmethod
    def _mode_response(changed: bool, detail: str) -> dict[str, object]:
        response = ControlRuntime._response(True, detail)
        response["changed"] = changed
        return response


class UnixControlServer:
    def __init__(self, socket_path: Path, runtime: ControlRuntime) -> None:
        self._socket_path = socket_path
        self._runtime = runtime
        self._should_stop = False

    def serve_forever(self) -> None:
        self._socket_path.parent.mkdir(parents=True, exist_ok=True)
        self._socket_path.unlink(missing_ok=True)
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            server.bind(str(self._socket_path))
            os.chmod(self._socket_path, 0o600)
            server.listen(8)
            while not self._should_stop:
                connection, _ = server.accept()
                with connection:
                    response = self._handle(connection)
                    connection.sendall(
                        (
                            json.dumps(response, separators=(",", ":")) + "\n"
                        ).encode("utf-8")
                    )
        finally:
            server.close()
            self._runtime.close()
            self._socket_path.unlink(missing_ok=True)

    def _handle(self, connection: socket.socket) -> dict[str, object]:
        try:
            request = json.loads(self._read_line(connection))
            if not isinstance(request, dict):
                raise ValueError("request must be a JSON object")
            operation = request["operation"]
            if not isinstance(operation, str):
                raise ValueError("operation must be a string")
            if operation == "shutdown":
                if set(request) != {"operation"}:
                    raise ValueError("shutdown request contains extra fields")
                self._should_stop = True
                return ControlRuntime._response(True, "runtime is shutting down")
            return self._runtime.execute(request)
        except (
            KeyError,
            RuntimeError,
            TypeError,
            ValueError,
            json.JSONDecodeError,
        ) as error:
            return ControlRuntime._response(False, f"request rejected: {error}")

    @staticmethod
    def _read_line(connection: socket.socket) -> str:
        payload = bytearray()
        while b"\n" not in payload:
            chunk = connection.recv(4096)
            if not chunk:
                break
            payload.extend(chunk)
            if len(payload) > MAX_MESSAGE_BYTES:
                raise ValueError("request exceeds 64 KiB")
        if not payload:
            raise ValueError("request is empty")
        return bytes(payload).split(b"\n", 1)[0].decode("utf-8")


def _integer(value: object, label: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError(f"{label} must be an integer")
    return value


def _require_request_keys(
    request: dict[str, Any],
    expected: set[str],
) -> None:
    if set(request) != expected:
        raise ValueError(
            f"request keys are invalid: expected={sorted(expected)}, "
            f"actual={sorted(request)}"
        )
