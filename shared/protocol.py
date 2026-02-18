"""Binary frame serialization for network transport.

Single-frame wire format:
    [4 bytes: rgb_compressed_len (big-endian uint32)]
    [rgb_compressed bytes]
    [depth_compressed bytes]

Batch wire format:
    [4 bytes: MAGIC 0x42415448 ("BATH")]
    [4 bytes: frame_count N]
    [4*N bytes: per-frame packed lengths]
    [concatenated pack_frame() outputs]

Both arrays are compressed with zlib level=1 for speed (~3-4x compression).
"""

import struct
import zlib

import numpy as np

_HEADER = struct.Struct(">I")  # 4-byte big-endian unsigned int
_BATCH_MAGIC = 0x42415448      # "BATH" in ASCII


def pack_frame(rgb: np.ndarray, depth: np.ndarray) -> bytes:
    """Compress and pack an RGB + depth frame pair into bytes."""
    rgb_bytes = zlib.compress(np.ascontiguousarray(rgb).tobytes(), level=1)
    depth_bytes = zlib.compress(np.ascontiguousarray(depth).tobytes(), level=1)
    return _HEADER.pack(len(rgb_bytes)) + rgb_bytes + depth_bytes


def unpack_frame(data: bytes) -> tuple[np.ndarray, np.ndarray]:
    """Unpack a compressed frame back into (rgb, depth) numpy arrays.

    rgb:   uint8  (480, 640, 3)
    depth: uint16 (480, 640)
    """
    (rgb_len,) = _HEADER.unpack_from(data, 0)
    offset = _HEADER.size
    rgb_compressed = data[offset : offset + rgb_len]
    depth_compressed = data[offset + rgb_len :]

    rgb = np.frombuffer(zlib.decompress(rgb_compressed), dtype=np.uint8).reshape(
        480, 640, 3
    )
    depth = np.frombuffer(zlib.decompress(depth_compressed), dtype=np.uint16).reshape(
        480, 640
    )
    return rgb, depth


def pack_frames(frames: list[tuple[np.ndarray, np.ndarray]]) -> bytes:
    """Pack multiple RGB+depth frame pairs into a single batch payload."""
    if not frames:
        raise ValueError("Cannot pack empty frame list")

    packed = [pack_frame(rgb, depth) for rgb, depth in frames]
    n = len(packed)

    # Header: magic + count + per-frame lengths
    parts = [_HEADER.pack(_BATCH_MAGIC), _HEADER.pack(n)]
    for p in packed:
        parts.append(_HEADER.pack(len(p)))
    parts.extend(packed)
    return b"".join(parts)


def unpack_frames(data: bytes) -> list[tuple[np.ndarray, np.ndarray]]:
    """Unpack a batch payload into a list of (rgb, depth) numpy arrays."""
    offset = 0
    (magic,) = _HEADER.unpack_from(data, offset)
    offset += 4
    if magic != _BATCH_MAGIC:
        raise ValueError(f"Invalid batch magic: 0x{magic:08X}")

    (n,) = _HEADER.unpack_from(data, offset)
    offset += 4

    lengths = []
    for _ in range(n):
        (length,) = _HEADER.unpack_from(data, offset)
        offset += 4
        lengths.append(length)

    frames = []
    for length in lengths:
        frames.append(unpack_frame(data[offset : offset + length]))
        offset += length

    return frames
