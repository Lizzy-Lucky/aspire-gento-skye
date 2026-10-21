"""Check Gento Skye's cuRobo collision spheres without a GPU.

Forward kinematics come from the URDF (yourdfpy). For a sphere set plus a self-collision ignore
list this reports:

* the number of sphere pairs cuRobo has to check (its fast self-collision kernel takes at most
  32 * 512 = 16384 pairs, beyond that it falls back to a slower kernel);
* overlapping link pairs in the reset pose (should be none);
* floor clearance of the lowest sphere in the reset pose (cuRobo counts anything within
  2 cm of an obstacle as a collision);
* for random configurations within the joint limits, how often each link pair overlaps. A pair
  that overlaps in almost every sample is touching by construction (sphere artefact or parts
  that sit against each other), not a collision the planner should avoid.

Usage (from /root/gpufree-data/aspire):

    ASPIRE/.venv-behavior/bin/python gento-skye-import/check_collision_spheres.py \
        --spheres gento-skye-import/fitted_collision_spheres.yaml --ignore proposed
"""

from __future__ import annotations

import argparse
import itertools
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml
import yourdfpy

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DESCRIPTION = ROOT / "gento_skye_description"
CUROBO_DEFAULT = (
    ROOT / "behavior-data/omnigibson-robot-assets/models/gento_skye/curobo"
    / "gento_skye_description_curobo_default.yaml"
)
FAST_KERNEL_PAIRS = 32 * 512
FLOOR_CLEARANCE_M = 0.03

# Order printed by the simulator (robot.joints), matching RESET_JOINT_POSITIONS in config.py.
SIM_JOINTS = [
    "base_footprint_x_joint", "base_footprint_y_joint", "base_footprint_z_joint",
    "base_footprint_rx_joint", "base_footprint_ry_joint", "base_footprint_rz_joint",
    "waist_joint_1", "waist_joint_2", "waist_joint_3", "Head_J1_Joint", "Joint1_L", "Joint1_R",
    "Head_J2_Joint", "Joint2_L", "Joint2_R", "Joint3_L", "Joint3_R", "Joint4_L", "Joint4_R",
    "Joint5_L", "Joint5_R", "Joint6_L", "Joint6_R", "Joint7_L", "Joint7_R",
    "Arm_L10_Joint", "Arm_L11_Joint", "Arm_R10_Joint", "Arm_R11_Joint",
]


def reset_joint_positions() -> dict[str, float]:
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "gento_behavior_config", ROOT / "ASPIRE/src/cap_harness/behavior/config.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # type: ignore[union-attr]
    return dict(zip(SIM_JOINTS, module.RESET_JOINT_POSITIONS))


def structural_ignore() -> set[frozenset[str]]:
    """Pairs that cannot collide whatever the joints do; everything else is checked."""
    pairs: set[frozenset[str]] = set()
    for side, wrist in (("L", "Link_Left_wrist"), ("R", "Link_Right_wrist")):
        fixed = [f"Link7_{side}", f"TCP_Link_{side}", f"Link8_{side}", wrist, f"Arm_{side}9_Link"]
        pairs |= {frozenset(p) for p in itertools.combinations(fixed, 2)}
        fingers = (f"Arm_{side}10_Link", f"Arm_{side}11_Link")
        for finger in fingers:
            pairs |= {frozenset((finger, other)) for other in fixed}
        pairs.add(frozenset(fingers))
    # Wheels, forks, casters and suspension sit inside the chassis and only spin or swivel.
    undercarriage = ["ZQ_Link", "ZQL_Link", "YQ_Link", "YQL_Link", "ZH_Link", "ZHL_Link",
                     "YH_Link", "YHL_Link", "ZLD_Link", "YLD_Link", "XJ_Link"] + [f"C{i}_Link" for i in range(1, 9)]
    pairs |= {frozenset(p) for p in itertools.combinations(["base_link", *undercarriage], 2)}
    # Measured: these overlap in every sampled configuration, Link5/Link7 by up to 8.5 cm with
    # spheres only 1.5 cm proud of the meshes (the wrist housings nest into each other), and the
    # two waist plates by at most 0.9 cm (sphere bulge). They touch by construction.
    pairs |= {frozenset(("Link5_L", "Link7_L")), frozenset(("Link5_R", "Link7_R")),
              frozenset(("waist_Link_1_prismatic", "waist_Link_3"))}
    return pairs


def load_ignore(kind: str, kin: dict) -> set[frozenset[str]]:
    current = {
        frozenset((a, b))
        for a, others in (kin.get("self_collision_ignore") or {}).items()
        for b in (others or [])
    }
    if kind == "current":
        return current
    # proposed: the importer's parent/child pairs plus structural pairs only.
    urdf = yourdfpy.URDF.load(str(DESCRIPTION / "urdf/gento_skye.urdf"), load_meshes=False, build_scene_graph=False)
    parent_child = {frozenset((j.parent, j.child)) for j in urdf.robot.joints}
    return parent_child | structural_ignore()


class Model:
    def __init__(self, spheres: dict[str, list[dict]]) -> None:
        self.urdf = yourdfpy.URDF.load(str(DESCRIPTION / "urdf/gento_skye.urdf"), load_meshes=False)
        self.links = [link for link in spheres if link in self.urdf.link_map]
        self.centers = {k: np.array([s["center"] for s in spheres[k]], dtype=np.float64) for k in self.links}
        self.radii = {k: np.array([s["radius"] for s in spheres[k]], dtype=np.float64) for k in self.links}
        self.joint_names = list(self.urdf.actuated_joint_names)
        self.limits = {}
        for name in self.joint_names:
            limit = self.urdf.joint_map[name].limit
            lo = limit.lower if limit is not None and limit.lower is not None else -np.pi
            hi = limit.upper if limit is not None and limit.upper is not None else np.pi
            self.limits[name] = (float(lo), float(hi))

    def world_spheres(self, q: dict[str, float]) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        cfg = np.array([q.get(name, 0.0) for name in self.joint_names])
        self.urdf.update_cfg(cfg)
        out = {}
        for link in self.links:
            T = self.urdf.get_transform(link, "base_link")
            c = self.centers[link] @ T[:3, :3].T + T[:3, 3]
            out[link] = (c, self.radii[link])
        return out


def overlaps(world, ignore, pairs_cache=None) -> dict[frozenset[str], float]:
    found: dict[frozenset[str], float] = {}
    links = list(world)
    for a, b in itertools.combinations(links, 2):
        key = frozenset((a, b))
        if key in ignore:
            continue
        ca, ra = world[a]
        cb, rb = world[b]
        depth = (ra[:, None] + rb[None, :]) - np.linalg.norm(ca[:, None] - cb[None], axis=-1)
        worst = float(depth.max())
        if worst > 0:
            found[key] = worst
    return found


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--spheres", default=None, help="yaml of link -> spheres (default: current cuRobo)")
    parser.add_argument("--ignore", choices=["current", "proposed"], default="proposed")
    parser.add_argument("--samples", type=int, default=2000)
    parser.add_argument("--never-close-samples", type=int, default=0)
    parser.add_argument("--never-close-margin", type=float, default=0.05)
    parser.add_argument("--write-never", default=None, help="save the never-close pairs to this yaml")
    args = parser.parse_args()

    kin = yaml.safe_load(CUROBO_DEFAULT.read_text())["robot_cfg"]["kinematics"]
    spheres = kin["collision_spheres"]
    if args.spheres:
        spheres = {**spheres, **yaml.safe_load(Path(args.spheres).read_text())}
    names = [n for n in kin["collision_link_names"] if not n.startswith("attached_object")]
    spheres = {n: spheres[n] for n in names if n in spheres}
    ignore = load_ignore(args.ignore, kin)
    model = Model(spheres)

    counts = {k: len(v) for k, v in spheres.items()}
    pairs = sum(counts[a] * counts[b] for a, b in itertools.combinations(model.links, 2) if frozenset((a, b)) not in ignore)
    print(f"spheres: {sum(counts.values())}   checked sphere pairs: {pairs}   fast-kernel limit: {FAST_KERNEL_PAIRS}")

    reset = reset_joint_positions()
    world = model.world_spheres(reset)
    lowest = min((c[:, 2] - r).min() for c, r in world.values())
    low_links = sorted(((float((c[:, 2] - r).min()), k) for k, (c, r) in world.items()))[:5]
    print(f"reset floor clearance: lowest sphere bottom {lowest * 100:.1f} cm (want >= {FLOOR_CLEARANCE_M * 100:.0f})")
    print("   lowest links:", ", ".join(f"{k} {z * 100:.1f}" for z, k in low_links))
    at_reset = overlaps(world, ignore)
    print(f"reset overlaps: {len(at_reset)}")
    for key, depth in sorted(at_reset.items(), key=lambda kv: -kv[1]):
        a, b = sorted(key)
        print(f"   {a} <-> {b}  {depth * 100:.1f} cm")

    rng = np.random.default_rng(0)
    movable = [n for n in model.joint_names if not n.startswith(("ZQ", "YQ", "ZH", "YH", "C"))]
    hits: dict[frozenset[str], list[float]] = defaultdict(list)
    for _ in range(args.samples):
        q = dict(reset)
        for name in movable:
            lo, hi = model.limits[name]
            q[name] = float(rng.uniform(lo, hi))
        for key, depth in overlaps(model.world_spheres(q), ignore).items():
            hits[key].append(depth)
    print(f"random samples: {args.samples}; link pairs overlapping in >= 90% of them:")
    always = [(key, len(v) / args.samples, max(v)) for key, v in hits.items() if len(v) / args.samples >= 0.9]
    for key, frac, worst in sorted(always, key=lambda t: -t[1]):
        a, b = sorted(key)
        print(f"   {a} <-> {b}  {frac * 100:.0f}%  max {worst * 100:.1f} cm")
    if not always:
        print("   none")

    if args.never_close_samples:
        never = never_close_pairs(model, reset, ignore, args.never_close_samples, args.never_close_margin)
        saved = sum(counts[a] * counts[b] for a, b in (sorted(p) for p in never))
        print(
            f"never within {args.never_close_margin * 100:.0f} cm in {args.never_close_samples} samples: "
            f"{len(never)} link pairs, {saved} sphere pairs -> {pairs - saved} left"
        )
        if args.write_never:
            out = sorted(sorted(p) for p in never)
            Path(args.write_never).write_text(yaml.safe_dump(out, default_flow_style=True))
            print("wrote", args.write_never)


def never_close_pairs(model: Model, reset: dict[str, float], ignore, samples: int, margin: float):
    """Link pairs whose closest spheres stay more than ``margin`` apart in every sample (MoveIt's
    "never in collision" rule). Samples cover the full joint limits plus each limit's extremes."""
    links = model.links
    index = np.concatenate([[i] * len(model.radii[k]) for i, k in enumerate(links)])
    radii = np.concatenate([model.radii[k] for k in links])
    gap_min = np.full((len(radii), len(radii)), np.inf)
    rng = np.random.default_rng(1)
    movable = [n for n in model.joint_names if not n.startswith(("ZQ", "YQ", "ZH", "YH", "C"))]
    for s in range(samples):
        q = dict(reset)
        for name in movable:
            lo, hi = model.limits[name]
            # A quarter of the samples push each joint to one of its limits.
            q[name] = float(rng.choice([lo, hi])) if rng.random() < 0.25 else float(rng.uniform(lo, hi))
        world = model.world_spheres(q)
        centers = np.concatenate([world[k][0] for k in links])
        gap = np.linalg.norm(centers[:, None] - centers[None], axis=-1) - radii[:, None] - radii[None]
        np.minimum(gap_min, gap, out=gap_min)
    never = set()
    for i, j in itertools.combinations(range(len(links)), 2):
        key = frozenset((links[i], links[j]))
        if key in ignore:
            continue
        block = gap_min[np.ix_(index == i, index == j)]
        if block.min() > margin:
            never.add(key)
    return never


if __name__ == "__main__":
    main()
