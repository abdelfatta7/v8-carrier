import numpy as np
import pytest

from v8carrier_navigation import map_io


def test_read_pgm_binary_and_ascii(tmp_path):
    pixels = np.array([[0, 128, 255], [254, 1, 100]], dtype=np.uint8)
    p5 = tmp_path / "m.pgm"
    map_io.write_pgm(str(p5), pixels)
    assert np.array_equal(map_io.read_pgm(str(p5)), pixels)
    p2 = tmp_path / "a.pgm"
    p2.write_text("P2\n# comment\n3 2\n255\n0 128 255\n254 1 100\n")
    assert np.array_equal(map_io.read_pgm(str(p2)), pixels)


def test_pixels_to_occupancy_trinary():
    pixels = np.array([[254, 205, 0, 100]], dtype=np.uint8)
    grid = map_io.pixels_to_occupancy(pixels)
    assert grid.tolist() == [[0, -1, 100, -1]]
    assert map_io.pixels_to_occupancy(pixels, negate=1).tolist() == [[100, 100, 0, -1]]  # 205/255 > 0.65


def test_load_map_flips_rows_and_reads_origin(tmp_path):
    # Image row 0 is the top of the map; the bottom row holds the free cell.
    pixels = np.full((4, 3), 0, dtype=np.uint8)
    pixels[3, 1] = 254
    map_io.write_pgm(str(tmp_path / "home.pgm"), pixels)
    (tmp_path / "home.yaml").write_text(
        "image: home.pgm\nresolution: 0.05\norigin: [-1.0, -2.0, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\n")
    m = map_io.load_map(str(tmp_path / "home.yaml"))
    assert m.resolution == 0.05
    assert m.origin == (-1.0, -2.0)
    assert m.grid.shape == (4, 3)
    assert m.grid[0, 1] == 0        # the free pixel is now in row 0 (bottom)
    assert (m.grid == 100).sum() == 11


def test_bad_magic_raises(tmp_path):
    (tmp_path / "x.pgm").write_bytes(b"P6\n1 1\n255\n\x00\x00\x00")
    with pytest.raises(ValueError):
        map_io.read_pgm(str(tmp_path / "x.pgm"))
