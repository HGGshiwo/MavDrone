from __future__ import annotations

from threading import Lock


class Buffer[T]:
    def __init__(self) -> None:
        self._item: T | None = None
        self._lock = Lock()

    def put(self, frame: T) -> None:
        with self._lock:
            self._item = frame

    def get(self) -> T | None:
        with self._lock:
            return self._item

    def has_frame(self) -> bool:
        with self._lock:
            return self._item is not None

    def clear(self) -> None:
        with self._lock:
            self._item = None
