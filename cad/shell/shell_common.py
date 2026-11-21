"""Shared geometry for the Cuby exterior shell (robot frame, mm).

The covers are built as "outer solid minus inner solid": a plan outline is
extruded vertically, its outer top edges are rounded, and the same outline
offset inward by the wall (same arc centres, radii +- wall) is subtracted.
Open sides (inner edge over the slot, front edge over the lid, rear edge of
the shroud) are then cut free with boxes.
"""
from __future__ import annotations

import math
import os

import yaml
from build123d import (Axis, Box, BuildLine, Cylinder, Edge, Line, Location,
                       Part, Plane, Pos, Rot, Solid, ThreePointArc, extrude,
                       make_face, mirror)

HERE = os.path.dirname(os.path.abspath(__file__))


def load_params(path: str | None = None) -> dict:
    with open(path or os.path.join(HERE, "params.yaml")) as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# primitives
def box(x0, x1, y0, y1, z0, z1) -> Part:
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def cyl_z(d, z0, z1, x, y) -> Part:
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(d / 2, z1 - z0)


def cyl_y(d, y0, y1, x, z) -> Part:
    y0, y1 = sorted((y0, y1))
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(d / 2, y1 - y0)


def prism_xz(pts, y0, y1) -> Part:
    """Prism from a polygon given as (x, z) points, spanning y0..y1."""
    from build123d import Polygon
    face = Plane.XZ * Polygon(*pts, align=None)
    solid = extrude(face, amount=y1 - y0, dir=(0, 1, 0))
    bb = solid.bounding_box()
    return Pos(0, y0 - bb.min.Y, 0) * solid


def round_rect_z(x0, x1, y0, y1, z0, z1, r) -> Part:
    """Vertical prism with rounded plan corners (cut-outs)."""
    b = box(x0, x1, y0, y1, z0, z1)
    r = min(r, (abs(x1 - x0)) / 2 - 0.01, abs(y1 - y0) / 2 - 0.01)
    if r > 0.2:
        b = b.fillet(r, b.edges().filter_by(Axis.Z))
    return b


def stadium_slot_z(x0, x1, y0, y1, z0, z1) -> Part:
    """Vertical-axis slot whose short ends are fully rounded."""
    w = min(abs(x1 - x0), abs(y1 - y0))
    return round_rect_z(x0, x1, y0, y1, z0, z1, w / 2)


# ---------------------------------------------------------------------------
def inner_face_offset(R, wall, clearance, rest_z, skin_z_in, corner, plan_corner_radius=None):
    """Smallest inner-face position (along the outward direction) so that the
    rounded inner face (edge radius R - wall) keeps `clearance` from a sharp
    edge at (corner, rest_z) of the part underneath (lid / nose top edge).

    With plan_corner_radius (outer, in plan) the same offset is applied on
    both sides of a sharp vertical corner of the part underneath, and it is
    grown until the corner blend (torus) and the vertical corner wall also
    keep the clearance from that corner."""
    r = R - wall
    c = clearance
    dz = rest_z - (skin_z_in - r)
    if dz <= 0:
        h = None
        a = c
    else:
        dmax = r - c
        if dmax <= dz:
            raise ValueError("skin_gap too small for this edge radius")
        h = math.sqrt(dmax ** 2 - dz ** 2)
        a = max(c, r - h)
    if plan_corner_radius is not None:
        rc = plan_corner_radius - wall
        if rc < r - 1e-6:
            raise ValueError("plan corner radius must be >= edge radius")
        a = max(a, rc - (rc - c) / math.sqrt(2))                 # vertical corner wall
        if h is not None:
            a = max(a, rc - (rc - r + h) / math.sqrt(2))          # corner blend vs the top corner
        a += 0.3                                                  # sampling / tessellation margin
    return corner + a


def _arc_pts(c, r, a0, a1):
    am = (a0 + a1) / 2
    f = lambda a: (c[0] + r * math.cos(math.radians(a)), c[1] + r * math.sin(math.radians(a)))
    return f(a0), f(am), f(a1)


def extrude_outline(segs, z0, z1) -> Part:
    """segs: list of ('L', p0, p1) or ('A', p0, pm, p1) in XY, closed loop."""
    with BuildLine() as bl:
        for s in segs:
            if s[0] == "L":
                Line(s[1], s[2])
            else:
                ThreePointArc(s[1], s[2], s[3])
    face = make_face(bl.edges())
    return Pos(0, 0, z0) * extrude(face, z1 - z0)


def fillet_top(solid: Part, radius: float, keep) -> Part:
    """Round the edges of the top face except those for which keep(edge) is True."""
    top = solid.faces().sort_by(Axis.Z)[-1]
    edges = [e for e in top.edges() if not keep(e)]
    return solid.fillet(radius, edges)


def edge_on_line(e: Edge, axis: str, value: float, tol=0.05) -> bool:
    i = {"x": 0, "y": 1}[axis]
    pts = [e.position_at(t) for t in (0.0, 0.5, 1.0)]
    return all(abs(tuple(p)[i] - value) < tol for p in pts)


def mirror_y(part: Part) -> Part:
    return mirror(part, about=Plane.XZ)


def solid_volume(part) -> float:
    return sum(s.volume for s in part.solids())
