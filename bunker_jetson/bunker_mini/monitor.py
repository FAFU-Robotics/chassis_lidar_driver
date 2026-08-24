"""Read-only CAN monitor for BUNKER MINI 2.0 feedback frames."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

import can

from .can_util import create_can_bus
from .protocol import (
    BmsFeedback,
    CanId,
    MotionFeedback,
    OdometerFeedback,
    RemoteControlFeedback,
    SystemStatus,
)


@dataclass
class RobotSnapshot:
    """Latest feedback collected from the chassis."""

    system_status: Optional[SystemStatus] = None
    motion: Optional[MotionFeedback] = None
    odometer: Optional[OdometerFeedback] = None
    bms: Optional[BmsFeedback] = None
    remote: Optional[RemoteControlFeedback] = None
    received_ids: set[int] = field(default_factory=set)
    all_frame_ids: set[int] = field(default_factory=set)
    total_frames: int = 0
    collected_at: float = field(default_factory=time.time)

    @property
    def is_connected(self) -> bool:
        return CanId.SYSTEM_STATUS in self.received_ids

    @property
    def has_bus_traffic(self) -> bool:
        return self.total_frames > 0


class BunkerMiniMonitor:
    """Listen to chassis feedback without sending motion commands."""

    _PARSERS = {
        CanId.SYSTEM_STATUS: SystemStatus.from_bytes,
        CanId.MOTION_FEEDBACK: MotionFeedback.from_bytes,
        CanId.ODOMETER: OdometerFeedback.from_bytes,
        CanId.BMS: BmsFeedback.from_bytes,
        CanId.REMOTE_CONTROL: RemoteControlFeedback.from_bytes,
    }

    _SNAPSHOT_FIELDS = {
        CanId.SYSTEM_STATUS: "system_status",
        CanId.MOTION_FEEDBACK: "motion",
        CanId.ODOMETER: "odometer",
        CanId.BMS: "bms",
        CanId.REMOTE_CONTROL: "remote",
    }

    def __init__(
        self,
        channel: str | None = None,
        interface: str | None = None,
        bustype_kwargs: Optional[dict] = None,
    ) -> None:
        self._bus = create_can_bus(
            channel=channel,
            interface=interface,
            bustype_kwargs=bustype_kwargs,
        )

    def close(self) -> None:
        self._bus.shutdown()

    def collect(self, duration_s: float = 2.0) -> RobotSnapshot:
        """Collect the latest value of each feedback frame within *duration_s*."""
        snapshot = RobotSnapshot()
        deadline = time.monotonic() + duration_s

        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            message = self._bus.recv(timeout=max(0.05, remaining))
            if message is None:
                continue
            self._update_snapshot(snapshot, message)

        snapshot.collected_at = time.time()
        return snapshot

    def watch(self, duration_s: Optional[float] = None, interval_s: float = 1.0) -> None:
        """Continuously refresh and yield snapshots until *duration_s* elapses."""
        started = time.monotonic()
        while True:
            yield self.collect(duration_s=interval_s)
            if duration_s is not None and time.monotonic() - started >= duration_s:
                break

    def _update_snapshot(self, snapshot: RobotSnapshot, message: can.Message) -> None:
        can_id = message.arbitration_id
        snapshot.total_frames += 1
        snapshot.all_frame_ids.add(can_id)

        parser = self._PARSERS.get(can_id)
        if parser is None:
            return
        field_name = self._SNAPSHOT_FIELDS[can_id]
        setattr(snapshot, field_name, parser(bytes(message.data)))
        snapshot.received_ids.add(can_id)
