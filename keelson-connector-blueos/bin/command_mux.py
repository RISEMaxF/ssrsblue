"""Command arbitration mux inspired by ROS twist_mux.

Multiple command sources are prioritized with timeouts. The highest-priority
active source wins. When a source times out, it falls through to the next
lower-priority active source. If all sources time out, neutral (0,0) is sent.
"""

import time
import logging
import threading

logger = logging.getLogger("command-mux")

# Default priorities: source_id prefix -> (priority, timeout_seconds)
# Higher priority wins. timeout=0 means never times out once seen.
DEFAULT_PRIORITIES = {
    "e_stop": (255, 0),
    "gamepad": (100, 0.5),
    "autonomy": (50, 2.0),
    "remote_ui": (10, 1.0),
}

DEFAULT_PRIORITY = 50


class CommandSource:
    __slots__ = ("source_id", "priority", "timeout", "last_seen",
                 "last_steering", "last_throttle")

    def __init__(self, source_id, priority, timeout):
        self.source_id = source_id
        self.priority = priority
        self.timeout = timeout
        self.last_seen = 0.0
        self.last_steering = 0.0
        self.last_throttle = 0.0

    @property
    def is_active(self):
        if self.last_seen == 0.0:
            return False
        if self.timeout <= 0:
            return True  # timeout=0 -> never times out (e.g. e_stop lock)
        return (time.monotonic() - self.last_seen) < self.timeout


class CommandMux:
    """Priority-based command source arbitration. Thread-safe."""

    def __init__(self, default_timeout=1.0):
        self.sources: dict[str, CommandSource] = {}
        self.active_source_id: str | None = None
        self.manual_override: str | None = None
        self.default_timeout = default_timeout
        self._lock = threading.Lock()

    def _get_or_create_source(self, source_id):
        if source_id not in self.sources:
            priority, timeout = self._lookup_priority(source_id)
            self.sources[source_id] = CommandSource(source_id, priority, timeout)
            logger.info("New source: %s (priority=%d, timeout=%.1fs)",
                        source_id, priority, timeout)
        return self.sources[source_id]

    def _lookup_priority(self, source_id):
        for prefix, (priority, timeout) in DEFAULT_PRIORITIES.items():
            if source_id.startswith(prefix):
                return priority, timeout
        return DEFAULT_PRIORITY, self.default_timeout

    def _update_active(self):
        """Determine active source. Must be called with lock held."""
        # Manual override takes precedence if that source is still active
        if self.manual_override:
            src = self.sources.get(self.manual_override)
            if src and src.is_active:
                old = self.active_source_id
                self.active_source_id = self.manual_override
                if old != self.active_source_id:
                    logger.info("Active source: %s (manual override)", self.active_source_id)
                return
            else:
                logger.info("Manual override %s expired", self.manual_override)
                self.manual_override = None

        # Highest-priority active source wins
        best = None
        for src in self.sources.values():
            if src.is_active:
                if best is None or src.priority > best.priority:
                    best = src

        old = self.active_source_id
        self.active_source_id = best.source_id if best else None
        if old != self.active_source_id:
            logger.info("Active source: %s -> %s", old, self.active_source_id)

    def on_manual_control(self, source_id, steering, throttle):
        with self._lock:
            src = self._get_or_create_source(source_id)
            src.last_seen = time.monotonic()
            src.last_steering = steering
            src.last_throttle = throttle
            self._update_active()

    def on_active_source(self, source_id):
        with self._lock:
            self.manual_override = source_id
            logger.info("Manual override requested: %s", source_id)
            self._update_active()

    def get_active_command(self):
        """Returns (source_id, steering, throttle) or None."""
        with self._lock:
            self._update_active()
            if self.active_source_id:
                src = self.sources[self.active_source_id]
                if src.is_active:
                    return src.source_id, src.last_steering, src.last_throttle
            return None
