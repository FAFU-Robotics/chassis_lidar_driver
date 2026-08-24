"""
MSOP packet extractor for RoboSense Airy PCAP files.

No pypcap / WinPcap / libpcap C headers required.

Backends (auto-selected by default):
  1. scapy ``rdpcap``  — ``pip install scapy`` (pure Python wheels on Windows)
  2. binary struct     — built-in fallback, zero extra dependencies
"""
from __future__ import annotations

import struct
from pathlib import Path
from typing import Iterator, List, Literal, Tuple

MSOP_MAGIC = bytes([0x55, 0xAA, 0x05, 0x5A])
MSOP_PACKET_SIZE = 1248
MSOP_PORT = 6699

MsopPacket = Tuple[float, bytes]  # (timestamp_sec, payload)

DEFAULT_PCAP = Path(__file__).with_name("airy_6x12_indoor(1).pcap")

Backend = Literal["auto", "scapy", "binary"]


def _parse_udp_payload(data: bytes, msop_port: int) -> bytes | None:
    """Extract UDP payload from an Ethernet/IPv4 frame (binary backend)."""
    if len(data) < 14:
        return None
    eth_type = struct.unpack("!H", data[12:14])[0]
    if eth_type != 0x0800:
        return None
    ip = data[14:]
    if len(ip) < 20 or (ip[0] >> 4) != 4:
        return None
    ihl = (ip[0] & 0x0F) * 4
    if ip[9] != 17 or len(ip) < ihl + 8:
        return None
    _sport, dport, ulen, _ = struct.unpack("!HHHH", ip[ihl:ihl + 8])
    if dport != msop_port:
        return None
    payload = ip[ihl + 8: ihl + ulen]
    if len(payload) != MSOP_PACKET_SIZE or payload[:4] != MSOP_MAGIC:
        return None
    return payload


def _load_msop_binary(pcap_path: Path, msop_port: int) -> List[MsopPacket]:
    """Parse classic PCAP via ``struct`` + ``open('rb')`` — no native libs."""
    packets: List[MsopPacket] = []
    with pcap_path.open("rb") as f:
        gh = f.read(24)
        if len(gh) < 24:
            raise ValueError(f"Invalid PCAP (too short): {pcap_path}")
        magic = struct.unpack("<I", gh[:4])[0]
        if magic == 0xA1B2C3D4:
            endian = "<"
        elif magic == 0xD4C3B2A1:
            endian = ">"
        else:
            raise ValueError(
                f"Unsupported PCAP format (magic={magic:#x}). "
                "Only classic PCAP is supported."
            )

        while True:
            ph = f.read(16)
            if len(ph) < 16:
                break
            ts_sec, ts_usec, incl, _orig = struct.unpack(endian + "IIII", ph)
            frame = f.read(incl)
            if len(frame) < incl:
                break
            payload = _parse_udp_payload(frame, msop_port)
            if payload is not None:
                packets.append((ts_sec + ts_usec * 1e-6, payload))
    return packets


def _load_msop_scapy(pcap_path: Path, msop_port: int) -> List[MsopPacket]:
    """Parse PCAP via scapy ``rdpcap`` (pure-Python pip package)."""
    from scapy.all import IP, UDP, rdpcap  # noqa: WPS433 — optional backend

    packets: List[MsopPacket] = []
    for pkt in rdpcap(str(pcap_path)):
        if UDP not in pkt:
            continue
        udp = pkt[UDP]
        if int(udp.dport) != msop_port:
            continue
        payload = bytes(udp.payload)
        if len(payload) != MSOP_PACKET_SIZE or payload[:4] != MSOP_MAGIC:
            continue
        packets.append((float(pkt.time), payload))
    return packets


def _resolve_backend(backend: Backend) -> str:
    if backend != "auto":
        return backend
    try:
        import scapy.all  # noqa: F401
        return "scapy"
    except ImportError:
        return "binary"


def load_msop_packets(pcap_path: str | Path,
                      msop_port: int = MSOP_PORT,
                      backend: Backend = "auto") -> List[MsopPacket]:
    """
    Load all MSOP packets from a PCAP file.

    Parameters
    ----------
    pcap_path : str | Path
        Classic ``.pcap`` file (not pcapng).
    msop_port : int
        UDP destination port (RoboSense MSOP default 6699).
    backend : ``"auto"`` | ``"scapy"`` | ``"binary"``
        ``auto`` tries scapy, falls back to binary struct parsing.

    Returns
    -------
    list of (timestamp_seconds, payload_1248_bytes)
    """
    path = Path(pcap_path)
    if not path.is_file():
        raise FileNotFoundError(f"PCAP not found: {path}")

    chosen = _resolve_backend(backend)
    if chosen == "scapy":
        try:
            packets = _load_msop_scapy(path, msop_port)
        except ImportError as exc:
            if backend == "scapy":
                raise ImportError(
                    "scapy is not installed. Run: pip install scapy"
                ) from exc
            packets = _load_msop_binary(path, msop_port)
            chosen = "binary"
    else:
        packets = _load_msop_binary(path, msop_port)

    if not packets:
        raise ValueError(
            f"No MSOP packets (port {msop_port}, {MSOP_PACKET_SIZE} B) "
            f"found in {path} (backend={chosen})"
        )
    return packets


def iter_msop_packets(pcap_path: str | Path,
                      msop_port: int = MSOP_PORT,
                      backend: Backend = "auto") -> Iterator[MsopPacket]:
    yield from load_msop_packets(pcap_path, msop_port, backend=backend)
