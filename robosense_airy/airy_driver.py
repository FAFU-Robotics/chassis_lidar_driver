"""
Interface for RoboSense Airy LiDAR (RSAIRY).
Pure-Python UDP driver for the WRS system — no ROS dependency.

Decoding logic ported from RoboSense rs_driver (decoder_RSAIRY.hpp):
  - MSOP packet: 1248 bytes, header 0x55 0xAA 0x05 0x5A, UDP port 6699
  - DIFOP packet: 1248 bytes, UDP port 7788 (angle calibration)
  - 3D conversion: x = r*cos(ω)*cos(α) + R*cos(α)
                  y = -r*cos(ω)*sin(α) - R*sin(α)
                  z = r*sin(ω) + Z

Author: WRS Lab
Requirement libs: 'numpy'
"""
from __future__ import annotations

import math
import socket
import statistics
import struct
import threading
import time
from collections import deque
from typing import Deque, List, Optional, Set, Tuple

import numpy as np

__VERSION__ = "0.1.0"

# ---------------------------------------------------------------------------
# Packet / decoder constants (from rs_driver decoder_RSAIRY.hpp)
# ---------------------------------------------------------------------------
MSOP_PORT = 6699
DIFOP_PORT = 7788
MSOP_PACKET_SIZE = 1248
DIFOP_PACKET_SIZE = 1248

MSOP_MAGIC = bytes([0x55, 0xAA, 0x05, 0x5A])
DIFOP_MAGIC = bytes([0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55])
BLOCK_MAGIC = bytes([0xFF, 0xEE])

CHANNEL_NUM = 96
BLOCKS_PER_PKT = 8
CHANNELS_PER_BLOCK = 48
DISTANCE_RES = 0.005  # metres per count
RS_ONE_ROUND = 36000  # 0.01-degree units per full rotation

# Lens-centre offset (metres)
RX = 0.0075
RY = 0.00664
RZ = 0.04532
LENS_CENTER_RXY = math.sqrt(RX * RX + RY * RY)

BLK_TS_US = 111.080
BLOCK_DURATION = BLK_TS_US / 1_000_000

# DIFOP struct offsets (RSAIRYDifopPkt, packed)
DIFOP_RPM_OFFSET = 8
DIFOP_FOV_START_OFFSET = 32
DIFOP_FOV_END_OFFSET = 34
DIFOP_INSTALL_MODE_OFFSET = 289
DIFOP_VERT_ANGLE_OFFSET = 468
DIFOP_HORIZ_ANGLE_OFFSET = 756

# MSOP struct offsets (RSAIRYMsopPkt, packed)
MSOP_HEADER_SIZE = 42
MSOP_BLOCK_SIZE = 148  # 2 + 2 + 48*3

# Upper bound on points per frame (96-ch Airy @ 10 Hz ≈ 50k–150k)
FRAME_POINT_CAP = 400_000

# Frame integrity / quality gate
# 1° azimuth bins → coverage == number of occupied bins (0..360)
AZ_BIN_DEG = 1
AZ_BIN_COUNT = 360 // AZ_BIN_DEG

# Quality tiers (publish policy). FULL bar stays at 300° — do NOT lower it
# just to reduce DROP counts. DEGRADED covers mild UDP-loss holes after a
# *confirmed* revolution boundary (see live logs: 282°/68k pts ≈ full 70k).
FULL_COVERAGE_DEG = 300
DEGRADED_COVERAGE_DEG = 270
# Alias kept for readability in older call sites / comments.
MIN_AZIMUTH_COVERAGE_DEG = FULL_COVERAGE_DEG

MIN_POINT_RATIO = 0.70
# DEGRADED allows slightly lower point ratio (holes remove slots when dense).
DEGRADED_POINT_RATIO = 0.65
GOOD_FRAME_HISTORY = 10
# Bootstrap floor before enough accepted frames exist for a median.
# Expected raw slots/rev ≈ packets/rev * 8 * 48 with TwoInOne (~0.4°/pair).
BOOTSTRAP_POINT_RATIO = 0.50

# Wrap candidate vs confirmed revolution boundary (0.01° units).
# Values MUST match the degree comments: unit = 0.01°, so 200° → 20000.
WRAP_MIN_JUMP = 30_000          # ≥ 180° backward jump → wrap *candidate*
ENDING_ZONE_MIN = 20_000        # high-water ≥ 200° to confirm end-of-rev
STARTING_ZONE_MAX = 10_000      # post-wrap angle ≤ 100° to confirm start-of-rev
WRAP_CONFIRM_COUNT = 2          # consecutive large-back samples (1: TwoInOne OK)

# Firing-time tables (microseconds) — ported from decoder_RSAIRY.hpp
FIRING_TSS_AIRY_M = [
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
    7.616, 7.616, 7.616, 7.616, 7.616, 7.616, 7.616, 7.616,
    16.184, 16.184, 16.184, 16.184, 16.184, 16.184, 16.184, 16.184,
    24.752, 24.752, 24.752, 24.752, 24.752, 24.752, 24.752, 24.752,
    33.320, 33.320, 33.320, 33.320, 33.320, 33.320, 33.320, 33.320,
    42.840, 42.840, 42.840, 42.840, 42.840, 42.840, 42.840, 42.840,
    52.360, 52.360, 52.360, 52.360, 52.360, 52.360, 52.360, 52.360,
    61.880, 61.880, 61.880, 61.880, 61.880, 61.880, 61.880, 61.880,
    71.400, 71.400, 71.400, 71.400, 71.400, 71.400, 71.400, 71.400,
    79.968, 79.968, 79.968, 79.968, 79.968, 79.968, 79.968, 79.968,
    88.536, 88.536, 88.536, 88.536, 88.536, 88.536, 88.536, 88.536,
    97.104, 97.104, 97.104, 97.104, 97.104, 97.104, 97.104, 97.104,
]

FIRING_TSS_SIDE = [
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
    11.424, 11.424, 11.424, 11.424, 11.424, 11.424, 11.424, 11.424,
    22.848, 22.848, 22.848, 22.848, 22.848, 22.848, 22.848, 22.848,
    34.272, 34.272, 34.272, 34.272, 34.272, 34.272, 34.272, 34.272,
    45.696, 45.696, 45.696, 45.696, 45.696, 45.696, 45.696, 45.696,
    54.264, 54.264, 54.264, 54.264, 54.264, 54.264, 54.264, 54.264,
    62.832, 62.832, 62.832, 62.832, 62.832, 62.832, 62.832, 62.832,
    71.400, 71.400, 71.400, 71.400, 71.400, 71.400, 71.400, 71.400,
    79.016, 79.016, 79.016, 79.016, 79.016, 79.016, 79.016, 79.016,
    85.680, 85.680, 85.680, 85.680, 85.680, 85.680, 85.680, 85.680,
    92.344, 92.344, 92.344, 92.344, 92.344, 92.344, 92.344, 92.344,
    99.008, 99.008, 99.008, 99.008, 99.008, 99.008, 99.008, 99.008,
]

FIRING_TSS_NORMAL = [
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
    5.712, 5.712, 5.712, 5.712, 5.712, 5.712, 5.712, 5.712,
    12.376, 12.376, 12.376, 12.376, 12.376, 12.376, 12.376, 12.376,
    19.040, 19.040, 19.040, 19.040, 19.040, 19.040, 19.040, 19.040,
    25.704, 25.704, 25.704, 25.704, 25.704, 25.704, 25.704, 25.704,
    33.320, 33.320, 33.320, 33.320, 33.320, 33.320, 33.320, 33.320,
    41.888, 41.888, 41.888, 41.888, 41.888, 41.888, 41.888, 41.888,
    50.456, 50.456, 50.456, 50.456, 50.456, 50.456, 50.456, 50.456,
    59.024, 59.024, 59.024, 59.024, 59.024, 59.024, 59.024, 59.024,
    70.448, 70.448, 70.448, 70.448, 70.448, 70.448, 70.448, 70.448,
    81.872, 81.872, 81.872, 81.872, 81.872, 81.872, 81.872, 81.872,
    93.296, 93.296, 93.296, 93.296, 93.296, 93.296, 93.296, 93.296,
]

FIRING_TSS_48L = [
    0.00, 0.00, 0.00, 0.00, 7.616, 7.616, 7.616, 7.616,
    16.184, 16.184, 16.184, 16.184, 24.752, 24.752, 24.752, 24.752,
    33.320, 33.320, 33.320, 33.320, 42.840, 42.840, 42.840, 42.840,
    52.360, 52.360, 52.360, 52.360, 61.880, 61.880, 61.880, 61.880,
    71.400, 71.400, 71.400, 71.400, 79.968, 79.968, 79.968, 79.968,
    88.536, 88.536, 88.536, 88.536, 97.104, 97.104, 97.104, 97.104,
]


# ---------------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------------

class _Trigon:
    """Lookup-table sin/cos for 0.01-degree angles (from rs_driver trigon.hpp)."""

    ANGLE_MIN = -9000
    ANGLE_MAX = 45000

    def __init__(self):
        angles = np.arange(self.ANGLE_MIN, self.ANGLE_MAX, dtype=np.float64) * 0.01
        rad = np.deg2rad(angles)
        self._sins = np.sin(rad).astype(np.float32)
        self._coss = np.cos(rad).astype(np.float32)

    def sin(self, angle: int) -> float:
        idx = angle - self.ANGLE_MIN
        if idx < 0 or idx >= len(self._sins):
            return 0.0
        return float(self._sins[idx])

    def cos(self, angle: int) -> float:
        idx = angle - self.ANGLE_MIN
        if idx < 0 or idx >= len(self._coss):
            return 0.0
        return float(self._coss[idx])


class _ChanAngles:
    """Vertical / horizontal angle calibration (from rs_driver chan_angles.hpp)."""

    def __init__(self, chan_num: int = CHANNEL_NUM):
        self.chan_num = chan_num
        self.vert_angles: List[int] = [0] * chan_num
        self.horiz_angles: List[int] = [0] * chan_num
        self.user_chans: List[int] = list(range(chan_num))
        self.ready = False

    def load_from_difop(self, packet: bytes) -> bool:
        vert, horiz = [], []
        for i in range(self.chan_num):
            off_v = DIFOP_VERT_ANGLE_OFFSET + i * 3
            sign_v = packet[off_v]
            if sign_v == 0xFF:
                return False
            val_v = struct.unpack_from(">H", packet, off_v + 1)[0]
            if sign_v != 0:
                val_v = -val_v
            if not (-9000 <= val_v < 18000):
                return False
            vert.append(val_v)

            off_h = DIFOP_HORIZ_ANGLE_OFFSET + i * 3
            sign_h = packet[off_h]
            val_h = struct.unpack_from(">H", packet, off_h + 1)[0]
            if sign_h != 0:
                val_h = -val_h
            if not (-9000 <= val_h < 18000):
                return False
            horiz.append(val_h)

        self.vert_angles = vert
        self.horiz_angles = horiz
        self._gen_user_chans()
        self.ready = True
        return True

    def load_default(self):
        """Approximate vertical angles when DIFOP is unavailable (~0.947 deg spacing)."""
        self.vert_angles = [int(i * 9000 / max(self.chan_num - 1, 1)) for i in range(self.chan_num)]
        self.horiz_angles = [0] * self.chan_num
        self._gen_user_chans()
        self.ready = True

    def _gen_user_chans(self):
        self.user_chans = []
        for i, angle in enumerate(self.vert_angles):
            self.user_chans.append(sum(1 for v in self.vert_angles if v < angle))

    def vert_adjust(self, chan: int) -> int:
        return self.vert_angles[chan]

    def horiz_adjust(self, chan: int, horiz: int) -> int:
        return horiz + round(self.horiz_angles[chan])


class _SplitByAngle:
    """
    Revolution-boundary detector with high-water tracking.

    observe() never lowers prev_angle on small regressions, and does not
    commit a wrap until the caller confirms the boundary.  This decouples:
      wrap *candidate*  (large backward jump)
      wrap *commit*     (caller decided this is a real frame boundary)
    """

    # observe() results
    FWD = "forward"
    SMALL_REGRESS = "small_regress"
    WRAP_CANDIDATE = "wrap_candidate"
    LATE = "late"  # previous-rev leftover after 0° wrap; do not mix

    def __init__(self, split_angle: int = 0,
                 wrap_min_jump: int = WRAP_MIN_JUMP,
                 wrap_confirm_count: int = WRAP_CONFIRM_COUNT,
                 ending_zone_min: int = ENDING_ZONE_MIN,
                 starting_zone_max: int = STARTING_ZONE_MAX):
        self.split_angle = split_angle
        self.prev_angle = split_angle
        self.high_water = split_angle
        self.wrap_min_jump = wrap_min_jump
        self.wrap_confirm_count = max(1, int(wrap_confirm_count))
        self.ending_zone_min = ending_zone_min
        self.starting_zone_max = starting_zone_max
        self._large_back_streak = 0
        self._pending_angle = 0
        self._pending_jump = 0

    def observe(self, angle: int) -> str:
        """
        Ingest one block azimuth.  Returns FWD / SMALL_REGRESS / WRAP_CANDIDATE.

        On WRAP_CANDIDATE, prev_angle / high_water are *unchanged* until
        commit_wrap() or reject_wrap().
        """
        angle = int(angle) % RS_ONE_ROUND

        if angle >= self.prev_angle:
            fwd_jump = angle - self.prev_angle
            # After a true 0° wrap, prev is small (e.g. 0.2°). A leftover
            # 359.x° block in the SAME MSOP (or a late UDP packet) is a
            # huge forward jump into the ending zone. Treating that as FWD
            # poisons high_water and then the rest of the new revolution
            # looks like another wrap → DROP with only tens of degrees.
            if (fwd_jump >= self.wrap_min_jump
                    and self.prev_angle <= self.starting_zone_max
                    and angle >= self.ending_zone_min):
                return self.LATE
            self.prev_angle = angle
            if angle > self.high_water:
                self.high_water = angle
            self._large_back_streak = 0
            return self.FWD

        jump = self.prev_angle - angle

        if jump < self.wrap_min_jump:
            # UDP reorder / jitter / duplicate — keep high-water prev.
            self._large_back_streak = 0
            return self.SMALL_REGRESS

        self._large_back_streak += 1
        self._pending_angle = angle
        self._pending_jump = jump
        if self._large_back_streak < self.wrap_confirm_count:
            return self.SMALL_REGRESS  # not enough confirms yet; don't move prev

        return self.WRAP_CANDIDATE

    def is_confirmed_boundary(self) -> bool:
        """True when pending candidate looks like end-of-rev → start-of-rev."""
        crosses_split = (
            self.high_water >= self.ending_zone_min
            and self._pending_angle < 18_000  # post-wrap < 180°
        )
        return bool(
            crosses_split
            and self._pending_jump >= self.wrap_min_jump
        )

    def commit_wrap(self) -> int:
        """Accept boundary: start new revolution at pending angle. Returns it."""
        angle = self._pending_angle
        self.prev_angle = angle
        self.high_water = angle
        self._large_back_streak = 0
        # clear old wrap candidate
        self._pending_angle = 0
        self._pending_jump = 0
        return angle

    def reject_wrap(self) -> None:
        """Reject candidate: keep high-water prev (no false rewind)."""
        self._large_back_streak = 0

    # Back-compat shim used by older unit tests / callers.
    def new_block(self, angle: int) -> bool:
        result = self.observe(angle)
        if result != self.WRAP_CANDIDATE:
            return False
        if self.is_confirmed_boundary():
            self.commit_wrap()
            return True
        self.reject_wrap()
        return False


def _build_two_in_one_iterator(azimuths: List[int], block_az_diff: int,
                                fov_blind_ts_diff: float) -> Tuple[List[int], List[float]]:
    """Block azimuth diff and timestamp offset (TwoInOneBlockIterator)."""
    az_diffs = [0] * BLOCKS_PER_PKT
    tss = [0.0] * BLOCKS_PER_PKT
    ts = 0.0
    blk = 0
    while blk < BLOCKS_PER_PKT - 2:
        ts_diff = BLOCK_DURATION
        az_diff = azimuths[blk + 2] - azimuths[blk]
        if az_diff < 0:
            az_diff += RS_ONE_ROUND
        if az_diff > 100:
            az_diff = block_az_diff
            ts_diff = fov_blind_ts_diff
        az_diffs[blk] = az_diffs[blk + 1] = az_diff
        tss[blk] = tss[blk + 1] = ts
        ts += ts_diff
        blk += 2
    az_diffs[blk] = az_diffs[blk + 1] = block_az_diff
    tss[blk] = tss[blk + 1] = ts
    return az_diffs, tss


def _distance_to_xyz(distance: float, angle_vert: int, angle_horiz: int,
                     trigon: _Trigon) -> Tuple[float, float, float]:
    """
    Polar → Cartesian (decoder_RSAIRY internDecodeMsopPkt).
    Angles in 0.01-degree integer units.
    """
    cos_v = trigon.cos(angle_vert)
    sin_v = trigon.sin(angle_vert)
    cos_h = trigon.cos(angle_horiz)
    sin_h = trigon.sin(angle_horiz)
    x = distance * cos_v * cos_h + LENS_CENTER_RXY * cos_h
    y = -distance * cos_v * sin_h - LENS_CENTER_RXY * sin_h
    z = distance * sin_v + RZ
    return x, y, z


# ---------------------------------------------------------------------------
# MSOP decoder
# ---------------------------------------------------------------------------

class _AiryMsopDecoder:
    """Decode RSAIRY MSOP packets into 3-D points."""

    def __init__(self, min_distance: float = 0.2, max_distance: float = 200.0,
                 dense_points: bool = False, split_angle: float = 0.0,
                 install_mode: Optional[int] = None,
                 verbose: bool = False):
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.dense_points = dense_points
        self.split_angle = int(split_angle * 100)
        self._verbose = verbose

        self.trigon = _Trigon()
        self.chan_angles = _ChanAngles(CHANNEL_NUM)
        self.split_strategy = _SplitByAngle(self.split_angle)

        self.chan_tss = [t / 1_000_000 for t in FIRING_TSS_AIRY_M]
        self.chan_azis = [t / BLK_TS_US for t in FIRING_TSS_AIRY_M]

        self.rps = 10
        self.block_az_diff = 40
        self.fov_blind_ts_diff = 0.0
        self.install_mode = install_mode
        self._decoder_ready = False
        self._echo_dual = False

        self._frame_points = np.empty((FRAME_POINT_CAP, 3), dtype=np.float32)
        self._frame_intensity = np.empty(FRAME_POINT_CAP, dtype=np.uint8)
        self._frame_count = 0
        self._completed_frame: Optional[Tuple[np.ndarray, np.ndarray]] = None

        # Per-revolution integrity accumulators
        self._frame_packet_count = 0
        self._frame_block_count = 0
        self._frame_az_bins: Set[int] = set()
        self._frame_suspect = False
        self._good_point_history: Deque[int] = deque(maxlen=GOOD_FRAME_HISTORY)
        self._dropped_frame_count = 0
        self._accepted_frame_count = 0
        self._full_frame_count = 0
        self._degraded_frame_count = 0
        self._last_frame_coverage = 0
        self._last_frame_point_count = 0
        self._last_frame_quality = "NONE"
        self._az_history: Deque[int] = deque(maxlen=10)

    def set_install_mode(self, mode: int):
        self.install_mode = mode
        self._decoder_ready = False

    def apply_difop(self, packet: bytes):
        """Parse DIFOP for RPM, FOV, install mode and angle calibration."""
        rpm = struct.unpack_from(">H", packet, DIFOP_RPM_OFFSET)[0]
        self.rps = rpm / 60 if rpm > 0 else 10
        self.block_az_diff = round(RS_ONE_ROUND * self.rps * BLOCK_DURATION)

        fov_start = struct.unpack_from(">H", packet, DIFOP_FOV_START_OFFSET)[0]
        fov_end = struct.unpack_from(">H", packet, DIFOP_FOV_END_OFFSET)[0]
        fov_range = (fov_end - fov_start) if fov_start < fov_end else (fov_end + RS_ONE_ROUND - fov_start)
        fov_blind = RS_ONE_ROUND - fov_range
        self.fov_blind_ts_diff = fov_blind / (RS_ONE_ROUND * self.rps)

        install_mode = packet[DIFOP_INSTALL_MODE_OFFSET]
        if install_mode != 0xFF:
            self.install_mode = install_mode

        if self.chan_angles.load_from_difop(packet):
            print("[Airy] DIFOP angle calibration loaded")
            self._decoder_ready = False

    def ensure_angles(self, wait_for_difop: bool):
        if not self.chan_angles.ready:
            if wait_for_difop:
                return False
            self.chan_angles.load_default()
        return True

    def _init_firing_table(self, lidar_mode: int, data_type_echo: int):
        """Select firing-time table based on channel count and install mode."""
        self._echo_dual = (data_type_echo == 0x03)

        if lidar_mode == 0x01:  # 48-channel
            table = FIRING_TSS_48L
        elif lidar_mode == 0x03:  # 192-channel — not fully supported
            table = FIRING_TSS_AIRY_M
        else:  # 96-channel (default)
            if self.install_mode == 0x1:
                table = FIRING_TSS_SIDE
            elif self.install_mode == 0x0:
                table = FIRING_TSS_NORMAL
            else:
                table = FIRING_TSS_AIRY_M

        self.chan_tss = [t / 1_000_000 for t in table]
        self.chan_azis = [t / BLK_TS_US for t in table]
        self._decoder_ready = True

    def _az_bin(self, block_az: int) -> int:
        """Map 0.01°-unit azimuth to a 1° occupancy bin in [0, 360)."""
        return (block_az // (100 * AZ_BIN_DEG)) % AZ_BIN_COUNT

    def _expected_points_per_rev(self) -> int:
        """Geometric estimate of raw point slots for one full revolution."""
        # TwoInOne: 4 azimuth steps per MSOP packet, each ≈ block_az_diff.
        az_per_pkt = max(4 * max(int(self.block_az_diff), 1), 1)
        pkts_per_rev = max(1, RS_ONE_ROUND // az_per_pkt)
        return pkts_per_rev * BLOCKS_PER_PKT * CHANNELS_PER_BLOCK

    def _min_points_threshold(self, ratio: float = MIN_POINT_RATIO) -> int:
        if len(self._good_point_history) >= 3:
            median_pts = statistics.median(self._good_point_history)
            return int(ratio * median_pts)
        return int(BOOTSTRAP_POINT_RATIO * self._expected_points_per_rev())

    def _reset_frame_accumulators(self) -> None:
        self._frame_count = 0
        self._frame_packet_count = 0
        self._frame_block_count = 0
        self._frame_az_bins = set()
        self._frame_suspect = False

    def _finalize_frame(self) -> bool:
        """
        Grade a completed revolution: FULL / DEGRADED / DROP.

        Returns True when a publishable frame (FULL or DEGRADED) was stored.
        External API unchanged: only accepted frames reach _completed_frame.
        """
        point_count = self._frame_count
        packet_count = self._frame_packet_count
        block_count = self._frame_block_count
        coverage = len(self._frame_az_bins) * AZ_BIN_DEG

        self._last_frame_point_count = point_count
        self._last_frame_coverage = coverage

        quality = "DROP"
        reason: Optional[str] = None

        if self._frame_suspect:
            reason = "suspect"
        elif coverage >= FULL_COVERAGE_DEG:
            if point_count >= self._min_points_threshold(MIN_POINT_RATIO):
                quality = "FULL"
            else:
                reason = "low_points"
        elif coverage >= DEGRADED_COVERAGE_DEG:
            # Soft-complete: mild UDP holes after a confirmed boundary.
            if point_count >= self._min_points_threshold(DEGRADED_POINT_RATIO):
                quality = "DEGRADED"
            else:
                reason = "low_points"
        else:
            reason = "low_coverage"

        self._last_frame_quality = quality

        if quality == "DROP":
            self._dropped_frame_count += 1
            print(
                f"DROP reason={reason} "
                f"coverage={coverage} "
                f"points={point_count} "
                f"packets={packet_count} "
                f"blocks={block_count}"
            )
            if self._verbose:
                print(
                    f"[Airy frame] DROP points={point_count} packets={packet_count} "
                    f"blocks={block_count} coverage={coverage}/{AZ_BIN_COUNT} "
                    f"reason={reason}"
                )
            return False

        self._completed_frame = self._flush_frame()
        self._accepted_frame_count += 1
        self._good_point_history.append(point_count)
        if quality == "FULL":
            self._full_frame_count += 1
        else:
            self._degraded_frame_count += 1
        if self._verbose:
            print(
                f"[Airy frame] {quality} points={point_count} packets={packet_count} "
                f"blocks={block_count} coverage={coverage}/{AZ_BIN_COUNT}"
            )
        return True

    def decode_msop(self, packet: bytes) -> bool:
        """
        Decode one MSOP packet.  Returns True when a publishable frame
        (FULL or DEGRADED) is ready.
        """
        if len(packet) != MSOP_PACKET_SIZE or packet[:4] != MSOP_MAGIC:
            return False

        data_type_0 = packet[16]
        if data_type_0 != 0:
            return False

        if not self._decoder_ready:
            lidar_mode = packet[32]
            self._init_firing_table(lidar_mode, packet[17])

        # Parse block azimuths
        azimuths = []
        for blk in range(BLOCKS_PER_PKT):
            off = MSOP_HEADER_SIZE + blk * MSOP_BLOCK_SIZE + 2
            azimuths.append(struct.unpack_from(">H", packet, off)[0])

        az_diffs, block_tss = _build_two_in_one_iterator(
            azimuths, self.block_az_diff, self.fov_blind_ts_diff)

        frame_completed = False
        # Credit packet_count on first block that actually joins a frame,
        # so a packet straddling two revolutions is counted once per side.
        packet_credited = False

        for blk in range(BLOCKS_PER_PKT):
            block_off = MSOP_HEADER_SIZE + blk * MSOP_BLOCK_SIZE
  
            if packet[block_off:block_off + 2] != BLOCK_MAGIC:
                self._frame_suspect = True
                break

            block_az = azimuths[blk]
            block_az_diff = az_diffs[blk]
            self._az_history.append(block_az)

            decision = self.split_strategy.observe(block_az)

            if decision == _SplitByAngle.LATE:
                # Old-revolution leftover in a straddling MSOP. Skip only
                # this block; later new-rev blocks in the same packet are kept.
                continue

            if decision == _SplitByAngle.WRAP_CANDIDATE:
                if self._verbose:
                    print(
                        f"BOUNDARY candidate: "
                        f"current={block_az/100:.1f} "
                        f"high={self.split_strategy.high_water/100:.1f} "
                        f"pending={self.split_strategy._pending_angle/100:.1f} "
                        f"jump={self.split_strategy._pending_jump/100:.1f}"
                    )
                if self._verbose:
                    hist = [f"{a / 100.0:.1f}" for a in self._az_history]
                    print(
                        f"[Airy wrap debug] current={block_az / 100.0:.1f} "
                        f"high_water={self.split_strategy.high_water / 100.0:.1f} "
                        f"pending_angle={self.split_strategy._pending_angle / 100.0:.1f} "
                        f"az_hist_deg=[{', '.join(hist)}]"
                    )
                if self.split_strategy.is_confirmed_boundary():
                    # Confirmed revolution boundary: always close the old
                    # frame (prevents mixing two spins), then grade it.
                    if self._frame_count > 0:
                        if self._finalize_frame():
                            frame_completed = True
                    self._reset_frame_accumulators()
                    self.split_strategy.commit_wrap()
                    # This block (and later new-rev blocks in the same MSOP)
                    # belong to the new revolution — fall through and decode.
                    packet_credited = False
                else:
                    # Large jump but not a credible 0° crossing (reorder /
                    # mid-rev glitch). Keep high-water; skip this block so
                    # it cannot pollute or extend the current frame.
                    self.split_strategy.reject_wrap()
                    if self._verbose:
                        print(
                            f"[Airy frame] REJECT_WRAP az={block_az / 100.0:.1f}° "
                            f"high_water={self.split_strategy.high_water / 100.0:.1f}° "
                            f"coverage={len(self._frame_az_bins) * AZ_BIN_DEG}/"
                            f"{AZ_BIN_COUNT}"
                        )
                    continue

            elif decision == _SplitByAngle.SMALL_REGRESS:
                # Mild reorder: keep high-water. Still accept points — they
                # often fill holes in the *same* revolution.
                pass

            if not packet_credited:
                self._frame_packet_count += 1
                packet_credited = True

            self._frame_block_count += 1
            self._frame_az_bins.add(self._az_bin(block_az))

            for chan in range(CHANNELS_PER_BLOCK):
                chan_id = chan + 48 if (blk % 2 == 1) else chan
                if chan_id >= CHANNEL_NUM:
                    continue

                ch_off = block_off + 4 + chan * 3
                raw_dist = struct.unpack_from(">H", packet, ch_off)[0]
                intensity = packet[ch_off + 2]

                u16_dist = raw_dist & 0x3FFF
                distance = u16_dist * DISTANCE_RES

                angle_vert = self.chan_angles.vert_adjust(chan_id)
                angle_horiz_raw = block_az + int(block_az_diff * self.chan_azis[chan_id])
                angle_horiz = self.chan_angles.horiz_adjust(chan_id, angle_horiz_raw)

                in_range = self.min_distance <= distance <= self.max_distance
                if in_range:
                    x, y, z = _distance_to_xyz(distance, angle_vert, angle_horiz, self.trigon)
                    i = self._frame_count
                    if i < FRAME_POINT_CAP:
                        self._frame_points[i, 0] = x
                        self._frame_points[i, 1] = y
                        self._frame_points[i, 2] = z
                        self._frame_intensity[i] = intensity
                        self._frame_count += 1
                elif not self.dense_points:
                    i = self._frame_count
                    if i < FRAME_POINT_CAP:
                        self._frame_points[i, 0] = np.nan
                        self._frame_points[i, 1] = np.nan
                        self._frame_points[i, 2] = np.nan
                        self._frame_intensity[i] = 0
                        self._frame_count += 1

        return frame_completed

    def _flush_frame(self) -> Tuple[np.ndarray, np.ndarray]:
        n = self._frame_count
        return (self._frame_points[:n].copy(),
                self._frame_intensity[:n].copy())

    def pop_frame(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        frame = self._completed_frame
        self._completed_frame = None
        return frame

    def get_current_frame(self) -> Tuple[np.ndarray, np.ndarray]:
        return self._flush_frame()

    @property
    def dropped_frame_count(self) -> int:
        return self._dropped_frame_count

    @property
    def accepted_frame_count(self) -> int:
        return self._accepted_frame_count

    @property
    def full_frame_count(self) -> int:
        return self._full_frame_count

    @property
    def degraded_frame_count(self) -> int:
        return self._degraded_frame_count

    @property
    def last_frame_coverage(self) -> int:
        return self._last_frame_coverage

    @property
    def last_frame_point_count(self) -> int:
        return self._last_frame_point_count

    @property
    def last_frame_quality(self) -> str:
        return self._last_frame_quality


# ---------------------------------------------------------------------------
# Minimal UDP receiver — bind all interfaces, recvfrom(1248), print on hit
# ---------------------------------------------------------------------------

def _open_msop_socket(port: int) -> socket.socket:
    """Bind UDP on all NICs (host='') and listen for MSOP packets."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind(("", port))
    return sock


def sniff_msop(port: int = MSOP_PORT):
    """
    Smoke-test loop: recvfrom(1248), print immediately on 55 AA 05 5A header.
    Blocks forever until Ctrl-C.
    """
    sock = _open_msop_socket(port)
    print(f"[Airy] 全网卡监听 UDP :{port}，等待 MSOP 数据包 ...")
    while True:
        data, addr = sock.recvfrom(MSOP_PACKET_SIZE)
        if data[:4] == MSOP_MAGIC:
            pass
#            print("成功抓到雷达数据包！")


# ---------------------------------------------------------------------------
# Shared frame pipeline (live UDP + offline PCAP)
# ---------------------------------------------------------------------------

class _AirySourceBase(object):
    """Decoder, frame cache, and WRS poll_frame API shared by UDP / PCAP sources."""

    def __init__(self,
                 min_distance: float = 0.2,
                 max_distance: float = 200.0,
                 dense_points: bool = False,
                 split_angle: float = 0.0,
                 install_mode: Optional[int] = None,
                 verbose: bool = False,
                 display_max_points: Optional[int] = None,
                 ego_filter_radius: float = 0.5):
        self._verbose = verbose
        self._running = True
        self._display_max_points = display_max_points
        self._ego_filter_radius = ego_filter_radius
        self._dense_points = dense_points

        self._decoder = _AiryMsopDecoder(
            min_distance=min_distance,
            max_distance=max_distance,
            dense_points=dense_points,
            split_angle=split_angle,
            install_mode=install_mode,
            verbose=verbose,
        )
        #self._decoder.chan_angles.load_default()

        self._latest_pcd: Optional[np.ndarray] = None
        self._latest_intensity: Optional[np.ndarray] = None
        self._full_pcd: Optional[np.ndarray] = None
        self._full_intensity: Optional[np.ndarray] = None
        self._last_raw_count: int = 0
        self._frame_seq: int = 0
        self._thread: Optional[threading.Thread] = None

    def _feed_msop(self, data: bytes) -> None:
        if len(data) != MSOP_PACKET_SIZE or data[:4] != MSOP_MAGIC:
            return
        if self._decoder.decode_msop(data):
            frame = self._decoder.pop_frame()
            if frame is not None:
                self._publish_frame(*frame)

    def _publish_frame(self, pcd: np.ndarray, intensity: np.ndarray) -> None:
        self._full_pcd, self._full_intensity = pcd, intensity
        self._last_raw_count = len(pcd)
        if self._display_max_points is not None:
            from wrs.drivers.devices.robosense_airy.viz import prepare_display_frame
            pcd, intensity = prepare_display_frame(
                pcd, intensity,
                max_points=self._display_max_points,
                ego_radius=self._ego_filter_radius,
            )
        self._latest_pcd, self._latest_intensity = pcd, intensity
        self._frame_seq += 1
        if self._verbose:
            print(f"  → 完整帧 #{self._frame_seq}，"
                  f"{self._last_raw_count:,} raw → {len(pcd):,} show")

    def req_data(self) -> Tuple[np.ndarray, np.ndarray]:
        pcd, intensity = self._wait_frame()
        return pcd, intensity

    def get_pcd(self, return_intensity: bool = False,
                remove_nan: bool = True) -> np.ndarray:
        pcd, intensity = self._wait_frame()
        if remove_nan:
            mask = ~np.isnan(pcd).any(axis=1)
            pcd = pcd[mask]
            intensity = intensity[mask]
        if return_intensity:
            return pcd, intensity
        return pcd

    def get_pcd_intensity(self, remove_nan: bool = True
                          ) -> Tuple[np.ndarray, np.ndarray]:
        return self.get_pcd(return_intensity=True, remove_nan=remove_nan)

    def get_latest_frame(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        return self._latest_pcd, self._latest_intensity

    def get_full_frame(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Full-resolution frame before display subsampling."""
        return self._full_pcd, self._full_intensity

    def poll_frame(self, last_seq: int = 0, remove_nan: bool = True
                   ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], int]:
        if self._latest_pcd is None or self._frame_seq <= last_seq:
            return None, None, last_seq
        pcd = self._latest_pcd
        intensity = self._latest_intensity
        if remove_nan and not self._dense_points:
            mask = ~np.isnan(pcd).any(axis=1)
            pcd = pcd[mask]
            intensity = intensity[mask]
        return pcd, intensity, self._frame_seq

    @property
    def frame_seq(self) -> int:
        return self._frame_seq

    @property
    def last_raw_point_count(self) -> int:
        return self._last_raw_count

    @property
    def dropped_frame_count(self) -> int:
        return self._decoder.dropped_frame_count

    @property
    def accepted_frame_count(self) -> int:
        return self._decoder.accepted_frame_count

    @property
    def full_frame_count(self) -> int:
        return self._decoder.full_frame_count

    @property
    def degraded_frame_count(self) -> int:
        return self._decoder.degraded_frame_count

    @property
    def last_frame_coverage(self) -> int:
        return self._decoder.last_frame_coverage

    @property
    def last_frame_point_count(self) -> int:
        return self._decoder.last_frame_point_count

    @property
    def last_frame_quality(self) -> str:
        return self._decoder.last_frame_quality

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _wait_frame(self) -> Tuple[np.ndarray, np.ndarray]:
        deadline = time.time() + getattr(self, "_frame_timeout", 3.0)
        while time.time() < deadline:
            if self._latest_pcd is not None:
                return self._latest_pcd, self._latest_intensity
            time.sleep(0.01)
        raise TimeoutError(
            "No point-cloud frame received within timeout. "
            "Check lidar / PCAP source."
        )


# ---------------------------------------------------------------------------
# Public driver class — live UDP
# ---------------------------------------------------------------------------

class RoboSenseAiry(_AirySourceBase):
    """
    RoboSense Airy LiDAR driver for the WRS system.

    Usage (mirrors RealSense / Zivid interface style)::

        lidar = RoboSenseAiry()
        pcd = lidar.get_pcd()          # nx3 numpy array (metres)
        pcd, intensity = lidar.get_pcd(return_intensity=True)
        lidar.stop()

    Parameters
    ----------
    msop_port : int
        MSOP UDP port (default 6699).  Socket always binds ``host=''`` (all NICs).
    min_distance, max_distance : float
        Valid range filter in metres.
    dense_points : bool
        If True, discard out-of-range points instead of inserting NaN.
    split_angle : float
        Frame-split azimuth in degrees (default 0).
    install_mode : int or None
        0 = normal, 1 = side, 2 = Airy-M.  ``None`` = read from DIFOP.
    frame_timeout : float
        Maximum wait (seconds) for a complete frame in ``get_pcd()``.
    """

    def __init__(self,
                 msop_port: int = MSOP_PORT,
                 min_distance: float = 0.2,
                 max_distance: float = 200.0,
                 dense_points: bool = False,
                 split_angle: float = 0.0,
                 install_mode: Optional[int] = None,
                 frame_timeout: float = 1.0,
                 verbose: bool = False,
                 display_max_points: Optional[int] = None,
                 ego_filter_radius: float = 0.5):
        super().__init__(
            min_distance=min_distance,
            max_distance=max_distance,
            dense_points=dense_points,
            split_angle=split_angle,
            install_mode=install_mode,
            verbose=verbose,
            display_max_points=display_max_points,
            ego_filter_radius=ego_filter_radius,
        )
        self._frame_timeout = frame_timeout
        self._msop_port = msop_port

        self._sock = _open_msop_socket(msop_port)
        self._thread = threading.Thread(
            target=self._recv_loop, daemon=True, name="RoboSenseAiry-UDP")
        self._thread.start()
        if verbose:
            print(f"[Airy] 全网卡监听 UDP :{msop_port} ...")

    def _recv_loop(self):
        while self._running:
            try:
                data, _addr = self._sock.recvfrom(MSOP_PACKET_SIZE)
            except OSError:
                break
            if self._verbose:
                print("成功抓到雷达数据包")
            self._feed_msop(data)

    def stop(self) -> None:
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass
        super().stop()

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Offline PCAP replay
# ---------------------------------------------------------------------------

class RoboSenseAiryPcap(_AirySourceBase):
    """
    Replay MSOP packets from a PCAP file through the same decoder pipeline.

    Parameters
    ----------
    pcap_path : str
        Path to a classic PCAP containing UDP MSOP (port 6699, 1248 B).
    loop : bool
        Restart from the beginning when the file ends (default True).
    speed : float
        Playback speed multiplier (1.0 = real-time, 2.0 = 2× faster).
    msop_port : int
        UDP destination port to filter in the PCAP.
    """

    def __init__(self,
                 pcap_path: str,
                 loop: bool = True,
                 speed: float = 1.0,
                 msop_port: int = MSOP_PORT,
                 min_distance: float = 0.2,
                 max_distance: float = 200.0,
                 dense_points: bool = False,
                 split_angle: float = 0.0,
                 install_mode: Optional[int] = None,
                 frame_timeout: float = 3.0,
                 verbose: bool = False,
                 display_max_points: Optional[int] = None,
                 ego_filter_radius: float = 0.5,
                 pcap_backend: str = "auto"):
        from wrs.drivers.devices.robosense_airy.pcap_reader import load_msop_packets

        super().__init__(
            min_distance=min_distance,
            max_distance=max_distance,
            dense_points=dense_points,
            split_angle=split_angle,
            install_mode=install_mode,
            verbose=verbose,
            display_max_points=display_max_points,
            ego_filter_radius=ego_filter_radius,
        )
        self._frame_timeout = frame_timeout
        self._loop = loop
        self._speed = max(float(speed), 0.01)
        self._pcap_path = str(pcap_path)
        self._packets = load_msop_packets(
            pcap_path, msop_port=msop_port, backend=pcap_backend)

        self._thread = threading.Thread(
            target=self._replay_loop, daemon=True, name="RoboSenseAiry-PCAP")
        self._thread.start()
        if verbose:
            dur = self._packets[-1][0] - self._packets[0][0]
            print(f"[Airy PCAP] {len(self._packets):,} MSOP packets, "
                  f"{dur:.1f}s, loop={'on' if loop else 'off'}, "
                  f"speed={self._speed}×")

    @property
    def packet_count(self) -> int:
        return len(self._packets)

    @property
    def duration_sec(self) -> float:
        if len(self._packets) < 2:
            return 0.0
        return self._packets[-1][0] - self._packets[0][0]

    def _replay_loop(self) -> None:
        loop_idx = 0
        while self._running:
            loop_idx += 1
            if self._verbose and loop_idx > 1:
                print(f"[Airy PCAP] 循环播放 #{loop_idx} ...")
            prev_ts: Optional[float] = None
            for ts, pkt in self._packets:
                if not self._running:
                    return
                if prev_ts is not None:
                    delay = (ts - prev_ts) / self._speed
                    if delay > 0:
                        time.sleep(delay)
                prev_ts = ts
                self._feed_msop(pkt)
            if not self._loop:
                break
        if self._verbose:
            print("[Airy PCAP] 回放结束")

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="RoboSense Airy LiDAR")
    parser.add_argument("--msop-port", type=int, default=MSOP_PORT)
    parser.add_argument("--sniff", action="store_true",
                        help="Raw UDP smoke test (print only)")
    parser.add_argument("--pcap", type=str, default=None,
                        help="Offline PCAP replay (skips UDP)")
    args = parser.parse_args()

    if args.sniff:
        print(f"RoboSense Airy driver v{__VERSION__} — 原始 UDP 抓包测试")
        try:
            sniff_msop(port=args.msop_port)
        except KeyboardInterrupt:
            print("\nStopped.")
    elif args.pcap:
        from wrs.drivers.devices.robosense_airy.pcap_replay import run_viewer
        run_viewer(pcap_path=args.pcap)
    else:
        print(f"RoboSense Airy driver v{__VERSION__} — 启动 3D 可视化")
        print("提示: 使用  python wrs/drivers/devices/robosense_airy/example.py")
        from wrs.drivers.devices.robosense_airy.example import main as viz_main
        viz_main()
