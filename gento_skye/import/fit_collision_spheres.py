"""Fit cuRobo collision spheres to Gento Skye's link meshes and report how well they fit.

The importer wrote one bounding sphere per link, which inflates long or flat links by
several centimetres and makes neighbouring links overlap at rest. This script fits several
spheres per link with cuRobo's own ``fit_spheres_to_mesh`` and measures, per link:

* uncovered: how far the worst mesh surface point lies outside every sphere (want <= ~1 cm);
* bulge: how far sphere surfaces stick out beyond the mesh (95th percentile and max).

Usage (from /root/gpufree-data/aspire):

    ASPIRE/.venv-behavior/bin/python gento-skye-import/fit_collision_spheres.py            # report only
    ASPIRE/.venv-behavior/bin/python gento-skye-import/fit_collision_spheres.py --write    # also save

``--write`` stores the result in ``gento-skye-import/fitted_collision_spheres.yaml``; applying it
to the cuRobo configs is a separate step (``apply_collision_spheres.py``).
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import trimesh
import yaml
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DESCRIPTION = ROOT / "gento_skye_description"
CUROBO_DEFAULT = (
    ROOT / "behavior-data/omnigibson-robot-assets/models/gento_skye/curobo"
    / "gento_skye_description_curobo_default.yaml"
)
OUTPUT = HERE / "fitted_collision_spheres.yaml"

# Wheels keep one hand-sized sphere each (r = 0.04 m on the axle): a tight fit reaches the
# floor, and cuRobo counts anything within 2 cm of the floor as a collision.
HAND_DESIGNED = {"ZQL_Link", "YQL_Link", "ZHL_Link", "YHL_Link"}
HAND_WHEEL = [{"center": [0.0, 0.0, 0.013], "radius": 0.04}]
FLOOR_CLEARANCE_M = 0.03
"""Minimum height of any sphere's bottom above the floor in the reset pose (base_link z = 0)."""
# One sphere is already tight for tiny links (TCP marker, wrist marker).
SPHERE_SIZE_M = 0.05
MAX_SPHERES = 48
BULGE_M = 0.015
"""How far a sphere may stick out of the part."""
TOLERANCE_M = 0.01
"""How far any surface point may lie outside every sphere."""
PER_LINK = {
    "base_link": {"n_max": 40, "bulge": 0.02, "tolerance": 0.03},
    "waist_Link_1_prismatic": {"bulge": 0.015, "tolerance": 0.02},
    "waist_Link_3": {"bulge": 0.015, "tolerance": 0.015},
    "head_Link_2": {"bulge": 0.015, "tolerance": 0.015},
}
"""Chassis, column, torso and head sit away from the fingertips; a looser fit there keeps
cuRobo's self-collision check within its fast kernel (at most 16384 checked sphere pairs).
Arm and gripper links keep the default 1 cm tolerance."""


def mesh_path(link: str) -> Path:
    found = list((DESCRIPTION / "meshes").rglob(f"{link}.STL"))
    if not found:
        raise FileNotFoundError(link)
    return found[0]


def sphere_count(mesh: trimesh.Trimesh) -> int:
    extents = np.sort(mesh.bounding_box.extents)[::-1]
    # Roughly one sphere per SPHERE_SIZE_M along the two longest axes.
    n = math.ceil(extents[0] / SPHERE_SIZE_M) * max(1, math.ceil(extents[1] / (2 * SPHERE_SIZE_M)))
    return int(min(MAX_SPHERES, max(1, n)))


class Solid:
    """A link as the union of the convex hulls of its separate parts.

    The STL files are open shells made of many loose parts, so inside/outside tests on them are
    unreliable. Each part's convex hull is closed with outward normals; hollow covers become solid,
    which is what collision checking wants anyway.
    """

    def __init__(self, mesh: trimesh.Trimesh) -> None:
        parts = mesh.split(only_watertight=False) or [mesh]
        hulls = []
        for part in parts:
            if len(part.vertices) < 4:
                continue
            try:
                hulls.append(part.convex_hull)
            except Exception:  # degenerate (flat) part
                continue
        self.hulls = [h for h in hulls if h.volume > 1e-9] or [mesh.convex_hull]
        self.bounds = np.array([np.min([h.bounds[0] for h in self.hulls], axis=0),
                                np.max([h.bounds[1] for h in self.hulls], axis=0)])

    def contains(self, pts: np.ndarray, skip: int | None = None) -> np.ndarray:
        inside = np.zeros(len(pts), dtype=bool)
        for i, hull in enumerate(self.hulls):
            if i != skip:
                inside |= hull.contains(pts)
        return inside

    def outer_surface(self, n: int, seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
        """Samples on the boundary of the union, with outward normals."""
        areas = np.array([h.area for h in self.hulls])
        counts = np.maximum(1, np.round(n * areas / areas.sum()).astype(int))
        pts, nrm = [], []
        for i, (hull, count) in enumerate(zip(self.hulls, counts)):
            p, f = trimesh.sample.sample_surface(hull, int(count), seed=seed + i)
            keep = ~self.contains(p, skip=i)
            pts.append(p[keep])
            nrm.append(hull.face_normals[f][keep])
        return np.concatenate(pts), np.concatenate(nrm)


def _candidates(solid: Solid, surface: np.ndarray, normals: np.ndarray) -> np.ndarray:
    """Interior points to try as sphere centres: a grid inside the solid, plus points pushed
    inward from the surface (thin parts the grid misses)."""
    lo, hi = solid.bounds
    pitch = float(np.clip(min(hi - lo) / 4.0, 0.004, 0.02))
    axes = [np.arange(lo[i] + pitch / 2, hi[i], pitch) for i in range(3)]
    grid = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
    if len(grid) > 40000:
        grid = grid[np.random.default_rng(0).choice(len(grid), 40000, replace=False)]
    grid = grid[solid.contains(grid)] if len(grid) else grid
    pick = np.random.default_rng(1).choice(len(surface), min(len(surface), 3000), replace=False)
    pushed = np.concatenate([surface[pick] - normals[pick] * d for d in (0.004, 0.008, 0.015, 0.03)])
    pushed = pushed[solid.contains(pushed)]
    return np.concatenate([grid, pushed])


def fit(
    mesh: trimesh.Trimesh, n_max: int, bulge: float = BULGE_M, tolerance: float = TOLERANCE_M
) -> tuple[np.ndarray, np.ndarray]:
    """Greedy cover of the link's outer surface by spheres centred inside it.

    Each candidate's radius is its distance to the nearest surface sample plus ``bulge``, so a
    sphere sticks out of the part by about ``bulge`` at most. Spheres are added, each time the one
    that brings the most surface samples to within ``tolerance`` of some sphere, until every sample
    is within ``tolerance`` or ``n_max`` spheres are used.
    """
    solid = Solid(mesh)
    surface, normals = solid.outer_surface(8000)
    tree = cKDTree(surface)
    cand = _candidates(solid, surface, normals)
    if len(cand) == 0:
        return np.empty((0, 3)), np.empty(0)
    depth = tree.query(cand)[0]
    radius = depth + bulge
    reach = tree.query_ball_point(cand, radius + tolerance)
    pending = np.ones(len(surface), dtype=bool)  # samples farther than `tolerance` from all spheres
    centers, radii = [], []
    while len(centers) < n_max and pending.any():
        gains = np.array([np.count_nonzero(pending[idx]) if idx else 0 for idx in reach])
        best = int(np.argmax(gains))
        if gains[best] == 0:
            break
        centers.append(cand[best])
        radii.append(radius[best])
        pending[reach[best]] = False
    return np.asarray(centers, dtype=np.float64), np.asarray(radii, dtype=np.float64)


def quality(mesh: trimesh.Trimesh, centers: np.ndarray, radii: np.ndarray) -> dict[str, float]:
    """Measured against the same union-of-hulls solid the fit uses."""
    solid = Solid(mesh)
    surface, _ = solid.outer_surface(20000, seed=7)
    gap = np.min(np.linalg.norm(surface[:, None, :] - centers[None], axis=-1) - radii[None], axis=1)
    uncovered = float(max(0.0, gap.max()))
    tree = cKDTree(solid.outer_surface(40000, seed=11)[0])
    bulge = []
    dirs = np.random.default_rng(0).normal(size=(200, 3))
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    for c, r in zip(centers, radii):
        pts = c + r * dirs
        outside = ~solid.contains(pts)
        if outside.any():
            bulge.append(tree.query(pts[outside])[0])
    bulge_all = np.concatenate(bulge) if bulge else np.zeros(1)
    return {
        "uncovered": uncovered,
        "bulge_p95": float(np.percentile(bulge_all, 95)),
        "bulge_max": float(bulge_all.max()),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--links", nargs="*", help="only these links")
    parser.add_argument("--refit-hand", action="store_true", help="also fit the hand-designed links")
    parser.add_argument("--max-spheres", type=int, default=MAX_SPHERES)
    args = parser.parse_args()

    kin = yaml.safe_load(CUROBO_DEFAULT.read_text())["robot_cfg"]["kinematics"]
    links = [n for n in kin["collision_link_names"] if not n.startswith("attached_object")]
    if args.links:
        links = [n for n in links if n in args.links]

    result: dict[str, list[dict[str, object]]] = {}
    print(f"{'link':24s} {'size (cm)':>18s} {'old':>10s} {'n':>3s} {'uncov':>6s} {'bulge95':>7s} {'bulgeMax':>8s}")
    for link in links:
        mesh = trimesh.load(mesh_path(link), force="mesh")
        size = np.round(mesh.bounding_box.extents * 100, 1)
        old = kin["collision_spheres"].get(link, [])
        old_txt = f"{len(old)}x r{max(s['radius'] for s in old):.3f}" if old else "-"
        degenerate = float(mesh.bounding_box.extents.max()) < 0.005
        if degenerate or (link in HAND_DESIGNED and not args.refit_hand):
            # Marker links (TCP) have no real geometry; keep what the importer wrote.
            spheres = HAND_WHEEL if link in HAND_DESIGNED else old
            centers = np.array([s["center"] for s in spheres], dtype=np.float64)
            radii = np.array([s["radius"] for s in spheres], dtype=np.float64)
        else:
            params = {"n_max": args.max_spheres, **PER_LINK.get(link, {})}
            centers, radii = fit(mesh, params["n_max"], params.get("bulge", BULGE_M), params.get("tolerance", TOLERANCE_M))
            if len(centers) == 0:
                raise RuntimeError(f"no sphere fitted for {link}")
            if link == "base_link":
                # base_link's frame sits on the floor; keep every sphere's bottom clear of it.
                radii = np.minimum(radii, centers[:, 2] - FLOOR_CLEARANCE_M)
                keep = radii >= 0.01
                centers, radii = centers[keep], radii[keep]
            spheres = [
                {"center": [round(float(v), 4) for v in c], "radius": round(float(r), 4)}
                for c, r in zip(centers, radii)
            ]
        q = quality(mesh, centers, radii) if not degenerate else {"uncovered": 0.0, "bulge_p95": 0.0, "bulge_max": 0.0}
        print(
            f"{link:24s} {str(size.tolist()):>18s} {old_txt:>10s} {len(spheres):3d} "
            f"{q['uncovered'] * 100:5.1f}  {q['bulge_p95'] * 100:6.1f}  {q['bulge_max'] * 100:7.1f}"
            + ("   (hand)" if link in HAND_DESIGNED else "")
        )
        result[link] = spheres

    print("total spheres:", sum(len(v) for v in result.values()))
    if args.write:
        OUTPUT.write_text(yaml.safe_dump(result, sort_keys=False, default_flow_style=None))
        print("wrote", OUTPUT)


if __name__ == "__main__":
    main()
