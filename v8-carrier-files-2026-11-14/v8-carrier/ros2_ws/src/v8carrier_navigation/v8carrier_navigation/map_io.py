"""Read a saved Nav2 map (map_saver's .yaml + .pgm) into an occupancy grid
array with OccupancyGrid conventions (-1 unknown, 0 free, 100 occupied).
Pure Python + numpy; no image library needed for PGM."""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Tuple

import numpy as np
import yaml


@dataclass
class MapData:
    grid: np.ndarray                 # shape (height, width), int8, row 0 = bottom (y = origin_y)
    resolution: float
    origin: Tuple[float, float]      # x, y of the bottom-left corner (yaw is ignored)

    @property
    def origin_xy(self) -> Tuple[float, float]:
        return self.origin


def read_pgm(path: str) -> np.ndarray:
    """P5 (binary) or P2 (ASCII) PGM -> uint8/uint16 array, row 0 = top of the image."""
    with open(path, "rb") as f:
        data = f.read()

    tokens = []
    pos = 0
    # Header: magic, width, height, maxval, separated by whitespace, with # comments.
    while len(tokens) < 4:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            while pos < len(data) and data[pos:pos + 1] not in (b"\n", b"\r"):
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    pos += 1  # the single whitespace after maxval
    if magic == b"P5":
        dtype = np.uint8 if maxval < 256 else ">u2"
        arr = np.frombuffer(data, dtype=dtype, count=width * height, offset=pos)
        return arr.reshape(height, width).astype(np.uint16 if maxval >= 256 else np.uint8)
    if magic == b"P2":
        values = np.array(data[pos:].split(), dtype=np.uint16)[: width * height]
        return values.reshape(height, width)
    raise ValueError(f"not a PGM file: {path}")


def pixels_to_occupancy(pixels: np.ndarray, negate: int = 0, occupied_thresh: float = 0.65,
                        free_thresh: float = 0.196, maxval: int = 255) -> np.ndarray:
    """map_server's trinary interpretation of the image."""
    p = pixels.astype(np.float64) / maxval
    occ = p if negate else 1.0 - p
    grid = np.full(pixels.shape, -1, dtype=np.int8)
    grid[occ > occupied_thresh] = 100
    grid[occ < free_thresh] = 0
    return grid


def load_map(yaml_path: str) -> MapData:
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    image = meta["image"]
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    pixels = read_pgm(image)
    maxval = int(pixels.max()) if pixels.dtype != np.uint8 else 255
    grid = pixels_to_occupancy(pixels, int(meta.get("negate", 0)), float(meta.get("occupied_thresh", 0.65)),
                               float(meta.get("free_thresh", 0.196)), maxval)
    # The image's first row is the top of the map; OccupancyGrid row 0 is the bottom.
    grid = np.flipud(grid).copy()
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    return MapData(grid=grid, resolution=float(meta["resolution"]), origin=(float(origin[0]), float(origin[1])))


def write_pgm(path: str, pixels: np.ndarray) -> None:
    """P5 writer (used by tests and to save synthetic maps)."""
    h, w = pixels.shape
    with open(path, "wb") as f:
        f.write(f"P5\n# v8carrier\n{w} {h}\n255\n".encode())
        f.write(np.ascontiguousarray(pixels, dtype=np.uint8).tobytes())
