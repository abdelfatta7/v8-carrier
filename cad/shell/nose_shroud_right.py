"""nose_shroud_right: mirror of the left nose-shroud body; carries the two M3
heat-set inserts and the pin holes at the centre chin."""
from __future__ import annotations

from nose_shroud_left import body_left, split_features
from shell_common import load_params, mirror_y


def build(params: dict | None = None):
    p = params or load_params()
    shell, _ = body_left(p)
    return split_features(mirror_y(shell), p, "right")


NOTES = ("right half of the nose shroud (mirror); 2 x M3 x 5.7 heat-set inserts from the split face + 2 pin holes at the chin; "
         "keep the LD2410 window (front face, z < 125) uncovered")

if __name__ == "__main__":
    part = build()
    print(part.bounding_box(), part.volume)
