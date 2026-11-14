"""Shared context and helpers for the fit checks.

``Context`` loads ``cad/params.yaml`` once, builds every print piece and the
assembly (``cad.assembly.assembled``) in memory, loads ``cad/out/manifest.json``
and gives the checks a few geometry helpers:

* ``holes(piece)``: fastener features found from circular edges of the known
  diameters (insert 4.0, clearance 3.4, pin socket 4.1, counterbore 6.5,
  M5 pivot 5.3, 625ZZ seat 16, M2 2.0, M2.5 2.6), grouped into holes with an
  axis, a centre line and an extent along the axis.
* ``bbox_overlap`` for the pairwise prefilters.

Nothing here writes to ``cad/``.
"""

from __future__ import annotations

import collections
import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

from build123d import GeomType, Part, Vector

from cad import assembly
from cad.common import ROOT as CAD_ROOT
from cad.common import layout, load_params, max_piece

REPO = CAD_ROOT.parent
OUT = CAD_ROOT / "out"
CHECK_OUT = REPO / "checks" / "out"

PASS, FAIL, WARN, SKIP = "PASS", "FAIL", "WARN", "SKIP"

# radius -> feature kind (mm); filled from the parameters in Context
HOLE_KINDS = {
    "insert": ("fasteners", "insert_hole"),
    "clearance": ("fasteners", "clearance_hole"),
    "pin": ("fasteners_extra", "pin_socket_dia"),
    "cbore": ("tray", "nose_rim", "counterbore", "dia"),
    "m5": ("wheel_module", "pivot_bolt", "clearance"),
    "brg625": ("wheels", "pivot_bearing", "od"),
    "m2": ("fasteners_extra", "m2_hole"),
    "m25": ("fasteners_extra", "m25_hole"),
}


@dataclass
class CheckResult:
    name: str
    status: str = PASS
    summary: str = ""
    details: list[str] = field(default_factory=list)
    failures: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    data: dict = field(default_factory=dict)
    seconds: float = 0.0

    def fail(self, msg: str):
        self.failures.append(msg)
        self.status = FAIL

    def warn(self, msg: str):
        self.warnings.append(msg)
        if self.status == PASS:
            self.status = WARN

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "status": self.status,
            "summary": self.summary,
            "details": self.details,
            "failures": self.failures,
            "warnings": self.warnings,
            "data": self.data,
            "seconds": round(self.seconds, 1),
        }


@dataclass
class Hole:
    kind: str
    axis: str  # 'x', 'y' or 'z'
    uv: tuple[float, float]  # the two other coordinates of the axis line, in x, y, z order
    a0: float  # extent along the axis
    a1: float
    radius: float
    edges: int

    @property
    def depth(self) -> float:
        return self.a1 - self.a0

    def axis_point(self, a: float) -> tuple[float, float, float]:
        i = "xyz".index(self.axis)
        c = list(self.uv)
        c.insert(i, a)
        return tuple(c)

    def __str__(self) -> str:
        return f"{self.kind} {self.axis}-axis at {self.axis_label()} span {self.a0:.1f}..{self.a1:.1f}"

    def axis_label(self) -> str:
        others = [k for k in "xyz" if k != self.axis]
        return f"({others[0]}={self.uv[0]:.2f}, {others[1]}={self.uv[1]:.2f})"


def _dig(p: dict, path: tuple) -> float:
    v = p
    for k in path:
        v = v[k]
    return float(v)


class Context:
    """Everything the checks share.  Built once by run_all (or by a check run standalone)."""

    def __init__(self, build: bool = True):
        t0 = time.time()
        self.p = load_params()
        self.L = layout(self.p)
        self.max_piece = max_piece(self.p)
        self.manifest = json.loads((OUT / "manifest.json").read_text()) if (OUT / "manifest.json").exists() else {}
        self.pieces: dict[str, tuple[str, Part]] = {}
        self.assembled: dict[str, Part] = {}
        self.placeholder_masses: dict[str, float] = {}
        if build:
            self.pieces = assembly.build_pieces(self.p)
            self.assembled = assembly.assembled(self.p)
            self.placeholder_masses = assembly.placeholder_masses(self.p)
        self.hole_radii = {kind: _dig(self.p, path) / 2 for kind, path in HOLE_KINDS.items()}
        self._holes: dict[str, list[Hole]] = {}
        self.build_seconds = time.time() - t0

    # ---- piece lists --------------------------------------------------
    @property
    def print_pieces(self) -> list[str]:
        return list(self.pieces)

    def module_of(self, piece: str) -> str:
        return self.pieces[piece][0]

    def part(self, name: str) -> Part:
        """Print piece or assembled body by name."""
        if name in self.assembled:
            return self.assembled[name]
        return self.pieces[name][1]

    def stl_path(self, piece: str) -> Path:
        return OUT / f"{piece}.stl"

    # ---- hole features ------------------------------------------------
    def holes(self, name: str, tol: float = 0.01) -> list[Hole]:
        """Fastener holes of a body found from its circular edges."""
        if name in self._holes:
            return self._holes[name]
        part = self.part(name)
        groups: dict[tuple, list] = collections.defaultdict(list)
        for e in part.edges():
            if e.geom_type != GeomType.CIRCLE:
                continue
            r = e.radius
            kind = next((k for k, rk in self.hole_radii.items() if abs(r - rk) < tol), None)
            if kind is None:
                continue
            n = e.normal()
            comps = (abs(n.X), abs(n.Y), abs(n.Z))
            i = max(range(3), key=lambda k: comps[k])
            if comps[i] < 0.999:
                continue  # not axis-aligned; no such holes in this design
            c = e.arc_center
            cc = (c.X, c.Y, c.Z)
            uv = tuple(round(cc[k], 2) for k in range(3) if k != i)
            groups[(kind, "xyz"[i], uv)].append(cc[i])
        out = []
        for (kind, ax, uv), coords in groups.items():
            h = Hole(kind, ax, uv, round(min(coords), 3), round(max(coords), 3), self.hole_radii[kind], len(coords))
            # a circle of a hole diameter can also be the outline of a boss (e.g. the bumper's 16 mm pin
            # boss has the 625ZZ diameter): keep only real holes, i.e. the axis at mid depth is empty
            try:
                if part.is_inside(Vector(*h.axis_point((h.a0 + h.a1) / 2))):
                    continue
            except Exception:
                pass
            out.append(h)
        out.sort(key=lambda h: (h.kind, h.axis, h.uv))
        self._holes[name] = out
        return out


def bbox_of(part: Part):
    bb = part.bounding_box()
    return (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)


def bbox_overlap(a, b, tol: float = 0.0) -> bool:
    """Axis-aligned boxes (x0,y0,z0,x1,y1,z1) overlap or come within ``tol``."""
    return all(a[i] < b[i + 3] + tol and b[i] < a[i + 3] + tol for i in range(3))


def dist_axes(h1: Hole, h2: Hole) -> float:
    """Distance between two parallel axis lines (inf if not parallel)."""
    if h1.axis != h2.axis:
        return math.inf
    return math.hypot(h1.uv[0] - h2.uv[0], h1.uv[1] - h2.uv[1])


def fmt(v: float, nd: int = 1) -> str:
    return f"{v:.{nd}f}"


def write_json(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, default=str))
