#!/usr/bin/env python3
"""Entry point for the Conda ONNX/Unitree control process."""

from __future__ import annotations

from pathlib import Path
import socket
import subprocess
import sys

from .config import load_config


def run(config_file: str, package_root: str, socket_path: str) -> None:
    config = load_config(config_file, package_root)
    available_interfaces = {name for _, name in socket.if_nameindex()}
    interface = config.runtime.network_interface
    if interface not in available_interfaces:
        raise ValueError(
            f"network interface {interface!r} is unavailable; "
            f"available={sorted(available_interfaces)}"
        )
    if config.runtime.should_check_robot:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", "1", config.runtime.robot_ip],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if result.returncode != 0:
            raise ConnectionError(
                f"robot is unreachable: {config.runtime.robot_ip}"
            )

    from .runtime import ControlRuntime, UnixControlServer

    runtime = ControlRuntime(config)
    UnixControlServer(Path(socket_path), runtime).serve_forever()


def main() -> None:
    if len(sys.argv) != 4:
        raise SystemExit(
            "usage: python -m locomotion_controller.runtime_process "
            "<config-file> <package-root> <unix-socket>"
        )
    run(sys.argv[1], sys.argv[2], sys.argv[3])


if __name__ == "__main__":
    main()
