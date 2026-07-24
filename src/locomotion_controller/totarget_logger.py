"""Non-blocking JSONL logging for ToTarget replay sessions."""

from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
from queue import Full, Queue
from threading import Thread
from typing import Any


LOG_SCHEMA = "hecbot.totarget_replay.v1"
MAX_QUEUED_RECORDS = 4096


class ToTargetLogger:
    """Write ordered replay records without performing file I/O at 50 Hz."""

    def __init__(self, directory: Path) -> None:
        self._directory = directory.expanduser().resolve()
        self._directory.mkdir(parents=True, exist_ok=True)
        self._queue: Queue[tuple[str, object]] = Queue(
            maxsize=MAX_QUEUED_RECORDS
        )
        self._writer_error: str | None = None
        self._is_active = False
        self._session_frame = 0
        self._session_path: Path | None = None
        self._thread = Thread(
            target=self._write_loop,
            name="totarget-jsonl-writer",
            daemon=True,
        )
        self._thread.start()

    @property
    def session_path(self) -> Path | None:
        return self._session_path

    def start_session(
        self,
        initial_target: tuple[float, float, float],
        metadata: dict[str, Any],
    ) -> Path:
        if self._is_active:
            self.end_session("restarted")
        started_at = datetime.now().astimezone()
        filename = _session_filename(initial_target, started_at)
        path = self._directory / filename
        header = {
            "schema": LOG_SCHEMA,
            "event": "session_start",
            "started_at": started_at.isoformat(timespec="microseconds"),
            "initial_target": list(initial_target),
            **metadata,
        }
        self._enqueue(("open", (path, header)))
        self._is_active = True
        self._session_frame = 0
        self._session_path = path
        return path

    def write_frame(self, payload: dict[str, Any]) -> None:
        if not self._is_active:
            return
        record = {
            "schema": LOG_SCHEMA,
            "event": "frame",
            "session_frame": self._session_frame,
            **payload,
        }
        self._enqueue(("record", record))
        self._session_frame += 1

    def end_session(self, reason: str) -> None:
        if not self._is_active:
            return
        footer = {
            "schema": LOG_SCHEMA,
            "event": "session_end",
            "reason": reason,
            "frame_count": self._session_frame,
            "ended_at": datetime.now().astimezone().isoformat(
                timespec="microseconds"
            ),
        }
        self._enqueue(("close", footer))
        self._is_active = False

    def close(self) -> None:
        if self._is_active:
            self.end_session("controller_closed")
        self._enqueue(("stop", None))
        self._thread.join()
        self._raise_if_failed()

    def _enqueue(self, item: tuple[str, object]) -> None:
        self._raise_if_failed()
        try:
            self._queue.put_nowait(item)
        except Full as error:
            raise RuntimeError(
                "ToTarget log queue is full; refusing to drop control frames"
            ) from error

    def _raise_if_failed(self) -> None:
        if self._writer_error is not None:
            raise RuntimeError(
                f"ToTarget log writer failed: {self._writer_error}"
            )

    def _write_loop(self) -> None:
        output = None
        try:
            while True:
                operation, value = self._queue.get()
                if operation == "open":
                    if output is not None:
                        output.close()
                    path, header = value
                    output = path.open("x", encoding="utf-8", buffering=1)
                    _write_json_line(output, header)
                    continue
                if operation == "record":
                    if output is None:
                        raise RuntimeError("frame received without an open session")
                    _write_json_line(output, value)
                    continue
                if operation == "close":
                    if output is not None:
                        _write_json_line(output, value)
                        output.close()
                        output = None
                    continue
                if operation == "stop":
                    if output is not None:
                        output.close()
                    return
                raise RuntimeError(f"unknown log operation: {operation!r}")
        except Exception as error:
            self._writer_error = str(error)
            if output is not None:
                output.close()


def _session_filename(
    initial_target: tuple[float, float, float],
    started_at: datetime,
) -> str:
    dx, dy, yaw = initial_target
    timestamp = started_at.strftime("%Y%m%dT%H%M%S.%f%z")
    return (
        f"dx_{dx:+.6f}_dy_{dy:+.6f}_yaw_{yaw:+.6f}_"
        f"{timestamp}.jsonl"
    )


def _write_json_line(output: Any, value: object) -> None:
    output.write(
        json.dumps(
            value,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        )
        + "\n"
    )
