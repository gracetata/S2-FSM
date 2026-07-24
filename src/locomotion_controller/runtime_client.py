"""ROS-side client for the isolated ONNX/Unitree runtime."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import socket
import subprocess
from threading import Lock
from time import monotonic, sleep

from .config import PackageConfig
from .protocol import ArmCommand


MAX_RESPONSE_BYTES = 64 * 1024


class RuntimeClient:
    def __init__(self, config: PackageConfig) -> None:
        self._config = config
        self._process: subprocess.Popen[bytes] | None = None
        self._request_lock = Lock()

    def start(self) -> None:
        executable = self._config.runtime.python_executable
        if not executable.is_file():
            raise RuntimeError(f"runtime Python is unavailable: {executable}")
        socket_path = self._config.runtime.socket_path
        socket_path.unlink(missing_ok=True)
        environment = os.environ.copy()
        cyclonedds_home = self._config.runtime.cyclonedds_home
        environment["CYCLONEDDS_HOME"] = str(cyclonedds_home)
        environment["PYTHONNOUSERSITE"] = "1"
        environment["LD_LIBRARY_PATH"] = _prepend_path(
            str(cyclonedds_home / "lib"),
            environment["LD_LIBRARY_PATH"]
            if "LD_LIBRARY_PATH" in environment
            else "",
        )
        source_root = Path(__file__).resolve().parents[1]
        environment["PYTHONPATH"] = _prepend_path(
            str(source_root),
            environment["PYTHONPATH"] if "PYTHONPATH" in environment else "",
        )
        self._process = subprocess.Popen(
            [
                str(executable),
                "-m",
                "locomotion_controller.runtime_process",
                str(self._config.config_path),
                str(self._config.package_root),
                str(socket_path),
            ],
            env=environment,
            start_new_session=True,
        )
        self._wait_until_ready()

    def set_high_mode(self, value: int) -> bool:
        return self._require_mode_change("high_mode", value)

    def set_low_mode(self, value: int) -> bool:
        return self._require_mode_change("low_mode", value)

    def set_navigation(self, values: tuple[float, float, float]) -> None:
        self._require_success("navigation", {"values": list(values)})

    def set_arm(self, command: ArmCommand) -> None:
        self._require_success("arm", {"command": command.to_payload()})

    def status(self) -> dict[str, object]:
        return self.request("status")

    def request(
        self,
        operation: str,
        payload: dict[str, object] | None = None,
    ) -> dict[str, object]:
        process = self._process
        if process is None or process.poll() is not None:
            raise RuntimeError("control runtime is not running")
        request = {"operation": operation}
        if payload is not None:
            request.update(payload)
        encoded = (json.dumps(request, separators=(",", ":")) + "\n").encode(
            "utf-8"
        )
        with self._request_lock:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
                connection.settimeout(self._config.runtime.request_timeout_s)
                connection.connect(str(self._config.runtime.socket_path))
                connection.sendall(encoded)
                connection.shutdown(socket.SHUT_WR)
                response_bytes = bytearray()
                while True:
                    chunk = connection.recv(4096)
                    if not chunk:
                        break
                    response_bytes.extend(chunk)
                    if len(response_bytes) > MAX_RESPONSE_BYTES:
                        raise RuntimeError("runtime response exceeds 64 KiB")
        response = json.loads(response_bytes.decode("utf-8"))
        if not isinstance(response, dict):
            raise RuntimeError("runtime response is not a JSON object")
        return response

    def close(self) -> None:
        process = self._process
        if process is None:
            return
        if process.poll() is None:
            try:
                self.request("shutdown")
                process.wait(timeout=self._config.controller.stop_timeout_s + 2.0)
            except (
                ConnectionError,
                OSError,
                RuntimeError,
                subprocess.TimeoutExpired,
            ):
                try:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=2.0)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    process.terminate()
                    try:
                        process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2.0)
        self._process = None
        self._config.runtime.socket_path.unlink(missing_ok=True)

    def _require_success(
        self,
        operation: str,
        payload: dict[str, object] | None = None,
    ) -> dict[str, object]:
        response = self.request(operation, payload)
        if response["success"] is not True:
            raise RuntimeError(str(response["detail"]))
        return response

    def _require_mode_change(self, operation: str, value: int) -> bool:
        response = self._require_success(operation, {"value": value})
        changed = response.get("changed")
        if not isinstance(changed, bool):
            raise RuntimeError("runtime mode response is missing changed state")
        return changed

    def _wait_until_ready(self) -> None:
        deadline = monotonic() + self._config.runtime.startup_timeout_s
        last_error = "runtime socket is not ready"
        while monotonic() < deadline:
            process = self._process
            if process is None:
                break
            return_code = process.poll()
            if return_code is not None:
                raise RuntimeError(
                    f"control runtime exited during startup: {return_code}"
                )
            if self._config.runtime.socket_path.exists():
                try:
                    response = self.status()
                    if response["success"] is True:
                        return
                    last_error = str(response["detail"])
                except (ConnectionError, OSError, RuntimeError) as error:
                    last_error = str(error)
            sleep(0.05)
        raise TimeoutError(f"control runtime startup timed out: {last_error}")


def _prepend_path(value: str, existing: str) -> str:
    entries = [entry for entry in existing.split(":") if entry and entry != value]
    return ":".join([value, *entries])
