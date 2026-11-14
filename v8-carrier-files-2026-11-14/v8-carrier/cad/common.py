"""Shared helpers for the part modules.

Everything is in the ROBOT FRAME: X forward (toward the nose), Y left, Z up,
origin on the floor plane directly below the midpoint of the drive-wheel axle.

The helpers take the parameter dict ``p`` loaded from ``cad/params.yaml``
(see :func:`load_params`) so that no module hard-codes a dimension.
"""

from __future__ import annotations

import math
from pathlib import Path

import yaml
from build123d import (
    Align,
    Axis,
    Box,
    Cylinder,
    Part,
    Plane,
    Pos,
    Rot,
)

ROOT = Path(__file__).resolve().parent
EPS = 0.01  # tiny overlap so coincident faces fuse/cut cleanly


# --------------------------------------------------------------------------
# parameters
# --------------------------------------------------------------------------
class _StrictLoader(yaml.SafeLoader):
    """SafeLoader that rejects duplicate mapping keys (PyYAML silently keeps the last)."""


def _construct_mapping(loader, node):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if key in mapping:
            raise ValueError(f"duplicate key {key!r} in parameters")
        mapping[key] = loader.construct_object(value_node, deep=True)
    return mapping


_StrictLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping)


def load_params(path: Path | None = None) -> dict:
    with open(path or ROOT / "params.yaml") as fh:
        return yaml.load(fh, Loader=_StrictLoader)


def load_measurements(path: Path | None = None) -> dict:
    with open(path or ROOT / "measurements.yaml") as fh:
        return yaml.load(fh, Loader=_StrictLoader) or {}


# --------------------------------------------------------------------------
# derived layout numbers used by several modules
# --------------------------------------------------------------------------
def layout(p: dict) -> dict:
    """Key robot-frame coordinates derived from the parameters."""
    front = p["wheels"]["axle_from_front"]
    rear = front - p["body"]["length"]
    nose_front = front - p["bumper"]["depth"]
    nose_rear = nose_front - p["nose"]["length"]
    half_w = p["body"]["width"] / 2
    floor_z0 = p["body_levels"]["ground_clearance"]
    floor_z1 = floor_z0 + p["body"]["floor"]
    slot_half = p["slot"]["width"] / 2
    pod_inner = slot_half + p["slot"]["clearance_each_side"]
    pod_outer = pod_inner + p["pods"]["width"]
    pod_front = nose_rear - p["tray"]["nose_rim"]["thickness"]
    pod_rear = pod_front - p["pods"]["length"]
    wm = p["wheel_module"]
    wheel_y = (
        half_w
        + wm["bracket_thickness"]
        + wm["arm_gap"]
        + wm["arm_thickness"]
        + wm["hub_gap"]
        + p["wheels"]["width"] / 2
    )
    return dict(
        x_front=front,
        x_rear=rear,
        nose_front=nose_front,
        nose_rear=nose_rear,
        half_w=half_w,
        floor_z0=floor_z0,
        floor_z1=floor_z1,
        slot_half=slot_half,
        pod_inner=pod_inner,
        pod_outer=pod_outer,
        pod_front=pod_front,
        pod_rear=pod_rear,
        pod_z0=floor_z1,
        pod_z1=floor_z1 + p["pods"]["height"],
        axle_z=p["wheels"]["diameter"] / 2,
        wheel_y=wheel_y,
        track_width=2 * wheel_y,
        caster_x=rear + p["caster"]["position_from_rear"],
    )


def max_piece(p: dict) -> float:
    """Largest allowed dimension of a printed piece."""
    bed = min(p["printer"]["bed_x"], p["printer"]["bed_y"], p["printer"]["bed_z"])
    return min(p["frame"]["max_piece"], bed - 2 * p["printer"]["split_margin"])


# --------------------------------------------------------------------------
# primitive makers (all return Parts positioned in the robot frame)
# --------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1) -> Part:
    """Axis-aligned box from min/max coordinates (any order)."""
    x0, x1 = sorted((x0, x1))
    y0, y1 = sorted((y0, y1))
    z0, z1 = sorted((z0, z1))
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def cyl_z(d, z0, z1, x, y) -> Part:
    z0, z1 = sorted((z0, z1))
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(d / 2, z1 - z0)


def cyl_y(d, y0, y1, x, z) -> Part:
    y0, y1 = sorted((y0, y1))
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(d / 2, y1 - y0)


def cyl_x(d, x0, x1, y, z) -> Part:
    x0, x1 = sorted((x0, x1))
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(d / 2, x1 - x0)


def cyl(axis: str, d, a0, a1, u, v) -> Part:
    """Cylinder along ``axis`` ('x', 'y' or 'z') spanning a0..a1; (u, v) are the
    other two coordinates in x, y, z order."""
    if axis == "z":
        return cyl_z(d, a0, a1, u, v)
    if axis == "y":
        return cyl_y(d, a0, a1, u, v)
    if axis == "x":
        return cyl_x(d, a0, a1, u, v)
    raise ValueError(axis)


def stadium_z(length, width, z0, z1, x, y, along="x") -> Part:
    """Slot (stadium) extruded along Z, overall ``length`` along ``along``."""
    r = width / 2
    if along == "x":
        body = box(x - length / 2 + r, x + length / 2 - r, y - r, y + r, z0, z1)
        ends = [cyl_z(width, z0, z1, x - length / 2 + r, y), cyl_z(width, z0, z1, x + length / 2 - r, y)]
    else:
        body = box(x - r, x + r, y - length / 2 + r, y + length / 2 - r, z0, z1)
        ends = [cyl_z(width, z0, z1, x, y - length / 2 + r), cyl_z(width, z0, z1, x, y + length / 2 - r)]
    return body + ends


def stadium_y(length, width, y0, y1, x, z, along="x") -> Part:
    """Slot extruded along Y (a window in a wall whose normal is Y)."""
    r = width / 2
    if along == "x":
        body = box(x - length / 2 + r, x + length / 2 - r, y0, y1, z - r, z + r)
        ends = [cyl_y(width, y0, y1, x - length / 2 + r, z), cyl_y(width, y0, y1, x + length / 2 - r, z)]
    else:
        body = box(x - r, x + r, y0, y1, z - length / 2 + r, z + length / 2 - r)
        ends = [cyl_y(width, y0, y1, x, z - length / 2 + r), cyl_y(width, y0, y1, x, z + length / 2 - r)]
    return body + ends


def union(parts) -> Part:
    parts = [q for q in parts if q is not None]
    out = parts[0]
    if len(parts) > 1:
        out = out + parts[1:]
    return out


def mirror_y(part: Part) -> Part:
    """Mirror a part across the XZ plane (left <-> right)."""
    return part.mirror(Plane.XZ)


# --------------------------------------------------------------------------
# fastener features
# --------------------------------------------------------------------------
def insert_hole(p: dict, axis: str, face_pos: float, direction: int, u, v, depth=None) -> Part:
    """Hole for an M3 heat-set insert, starting at the face coordinate
    ``face_pos`` on ``axis`` and going ``depth`` into the material in the
    sign of ``direction``.  A small overshoot clears the face."""
    f = p["fasteners"]
    depth = f["insert_depth"] if depth is None else depth
    a0 = face_pos - direction * EPS * 50  # 0.5 mm outside the face
    a1 = face_pos + direction * depth
    return cyl(axis, f["insert_hole"], a0, a1, u, v)


def clearance_hole(p: dict, axis: str, a0, a1, u, v) -> Part:
    return cyl(axis, p["fasteners"]["clearance_hole"], a0, a1, u, v)


def boss(p: dict, axis: str, a0, a1, u, v, od=None) -> Part:
    """Solid boss cylinder (default OD from the parameters)."""
    return cyl(axis, od or p["fasteners"]["boss_od"], a0, a1, u, v)


def pin_socket(p: dict, axis: str, face_pos: float, direction: int, u, v) -> Part:
    """Socket for a locating dowel: pin_socket_dia x pin_socket_depth."""
    fx = p["fasteners_extra"]
    a0 = face_pos - direction * 0.5
    a1 = face_pos + direction * fx["pin_socket_depth"]
    return cyl(axis, fx["pin_socket_dia"], a0, a1, u, v)


def standoff(p: dict, x, y, z0, od=None, height=None, hole=None) -> tuple[Part, Part]:
    """Return (boss, hole) for a standoff rising from z0.  Hole is an M3 insert
    hole by default, or a plain hole of diameter ``hole``."""
    so = p["pods"]["standoff"]
    height = so["height"] if height is None else height
    b = cyl_z(od or p["fasteners"]["boss_od"], z0 - EPS, z0 + height, x, y)
    if hole is None:
        h = insert_hole(p, "z", z0 + height, -1, x, y)
    else:
        h = cyl_z(hole, z0 + 1, z0 + height + 0.5, x, y)
    return b, h


# --------------------------------------------------------------------------
# splitting
# --------------------------------------------------------------------------
def halves_y(part: Part, big=2000.0) -> tuple[Part, Part]:
    """Cut a part on the centreline plane Y=0 into (left: y>0, right: y<0)."""
    left = part & box(-big, big, 0, big, -big, big)
    right = part & box(-big, big, -big, 0, -big, big)
    return left, right


def split_centreline(part: Part, p: dict, bolt_xz, pin_xz, flange_w: float | None = None,
                     insert_side: str = "left") -> tuple[Part, Part]:
    """Split ``part`` on Y=0 and drill the split joint.

    The caller must already have modelled the flange/rib material on both
    sides of the plane (see e.g. floor_tray.py).  For every (x, z) in
    ``bolt_xz`` a horizontal (Y-axis) M3 bolt crosses the plane: the
    ``insert_side`` half gets an insert hole, the other half a clearance hole
    ``flange_w`` long (default: the whole flange width from params.tray).  For
    every (x, z) in ``pin_xz`` both halves get a dowel socket.
    """
    flange_w = p["tray"]["flange"]["width"] if flange_w is None else flange_w
    left, right = halves_y(part)
    left_cuts, right_cuts = [], []
    for x, z in bolt_xz:
        if insert_side == "left":
            left_cuts.append(insert_hole(p, "y", 0, +1, x, z))
            right_cuts.append(clearance_hole(p, "y", 0.5, -(flange_w + 1), x, z))
        else:
            right_cuts.append(insert_hole(p, "y", 0, -1, x, z))
            left_cuts.append(clearance_hole(p, "y", -0.5, flange_w + 1, x, z))
    for x, z in pin_xz:
        left_cuts.append(pin_socket(p, "y", 0, +1, x, z))
        right_cuts.append(pin_socket(p, "y", 0, -1, x, z))
    if left_cuts:
        left = left - left_cuts
    if right_cuts:
        right = right - right_cuts
    return left, right


def split_plane_x(part: Part, p: dict, x_split: float, bolt_yz, pin_yz, flange_w: float | None = None,
                  big=2000.0) -> tuple[Part, Part]:
    """Split ``part`` on the plane X = x_split into (front: x > x_split, rear).
    Bolts run along X; the front half gets the inserts."""
    flange_w = p["tray"]["flange"]["width"] if flange_w is None else flange_w
    front = part & box(x_split, big, -big, big, -big, big)
    rear = part & box(-big, x_split, -big, big, -big, big)
    front_cuts, rear_cuts = [], []
    for y, z in bolt_yz:
        front_cuts.append(insert_hole(p, "x", x_split, +1, y, z))
        rear_cuts.append(clearance_hole(p, "x", x_split + 0.5, x_split - (flange_w + 1), y, z))
    for y, z in pin_yz:
        front_cuts.append(pin_socket(p, "x", x_split, +1, y, z))
        rear_cuts.append(pin_socket(p, "x", x_split, -1, y, z))
    if front_cuts:
        front = front - front_cuts
    if rear_cuts:
        rear = rear - rear_cuts
    return front, rear


# --------------------------------------------------------------------------
# reporting
# --------------------------------------------------------------------------
def bbox(part: Part) -> dict:
    bb = part.bounding_box()
    return {
        "min": [round(bb.min.X, 2), round(bb.min.Y, 2), round(bb.min.Z, 2)],
        "max": [round(bb.max.X, 2), round(bb.max.Y, 2), round(bb.max.Z, 2)],
        "size": [round(bb.size.X, 2), round(bb.size.Y, 2), round(bb.size.Z, 2)],
    }


def fits_bed(part: Part, p: dict) -> bool:
    bb = part.bounding_box()
    return max(bb.size.X, bb.size.Y, bb.size.Z) <= max_piece(p) + 1e-6


def mass_g(part: Part, p: dict) -> float:
    return part.volume / 1000.0 * p["materials"]["petg_density"]


def belt_centre_distance(p: dict) -> float:
    """Centre distance of two equal GT2 pulleys joined by a closed belt."""
    belt = p["brush_roll"]["belt"]
    pitch = 2.0  # GT2
    pitch_circumference = belt["pulley_teeth"] * pitch
    return (p["nose"]["belt_length"] - pitch_circumference) / 2


def deg(a):
    return math.radians(a)
