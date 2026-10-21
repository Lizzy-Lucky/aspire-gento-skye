"""Write the fitted collision spheres and the rebuilt self-collision ignore list into the four
Gento Skye cuRobo configs and into the importer's source config.

Inputs (from fit_collision_spheres.py and check_collision_spheres.py):
    gento-skye-import/fitted_collision_spheres.yaml
    gento-skye-import/never_close_pairs.yaml

What changes:
* The undercarriage links (wheels, wheel forks, suspension, casters, XJ_Link) leave cuRobo's
  collision model. Their joints are not in cuRobo's joint space, so during planning they are
  rigid with base_link, whose spheres already cover them; R1 Pro has no wheel spheres either.
* Every remaining link gets its fitted spheres.
* self_collision_ignore is rebuilt from: URDF parent/child pairs, pairs inside a rigid group
  (fixed joints or joints cuRobo locks), pairs measured to touch in every sampled configuration,
  and pairs that never came within 5 cm in 10000 sampled configurations. Entries for cuRobo's
  attached-object links are kept as they were.

Usage (from /root/gpufree-data/aspire):

    ASPIRE/.venv-behavior/bin/python gento-skye-import/apply_collision_spheres.py
"""

from __future__ import annotations

import itertools
import shutil
import sys
from datetime import datetime
from pathlib import Path

import yaml
import yourdfpy

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import check_collision_spheres as check  # noqa: E402

CUROBO_DIR = ROOT / "behavior-data/omnigibson-robot-assets/models/gento_skye/curobo"
SOURCE = HERE / "gento_skye_source_config.yaml"
FITTED = HERE / "fitted_collision_spheres.yaml"
NEVER = HERE / "never_close_pairs.yaml"
BACKUP_ROOT = ROOT / "behavior-data/omnigibson-robot-assets/models/gento_skye/misc/curobo-backup-20261021"

UNDERCARRIAGE = {
    "ZQ_Link", "ZQL_Link", "YQ_Link", "YQL_Link", "ZH_Link", "ZHL_Link", "YH_Link", "YHL_Link",
    "ZLD_Link", "YLD_Link", "XJ_Link", *(f"C{i}_Link" for i in range(1, 9)),
}

SOURCE_COMMENTS = {
    "  collision_spheres:": [
        "  # Fitted by gento-skye-import/fit_collision_spheres.py to the union of convex hulls of each",
        "  # link's mesh parts: arm and gripper links within ~1 cm (spheres stick out <= 1.5 cm), torso",
        "  # and head within 1.5-2 cm, base_link within 3 cm with every sphere bottom >= 3 cm above the",
        "  # floor. Undercarriage links (wheels, forks, casters) have no spheres: cuRobo does not move",
        "  # their joints, and base_link covers them. Check with check_collision_spheres.py.",
    ],
    "  self_collision_ignore:": [
        "  # Rebuilt by apply_collision_spheres.py: URDF parent/child pairs, pairs inside a rigid group",
        "  # (fixed or cuRobo-locked joints), Link5-Link7 and the two waist plates (measured to touch in",
        "  # every sampled configuration), and pairs never within 5 cm in 10000 sampled configurations.",
    ],
}


def rebuilt_ignore(names: list[str]) -> dict[str, list[str]]:
    keep = set(names)
    urdf = yourdfpy.URDF.load(str(check.DESCRIPTION / "urdf/gento_skye.urdf"), load_meshes=False, build_scene_graph=False)
    pairs = {frozenset((j.parent, j.child)) for j in urdf.robot.joints}
    pairs |= check.structural_ignore()
    pairs |= {frozenset(p) for p in yaml.safe_load(NEVER.read_text())}
    out: dict[str, list[str]] = {}
    for pair in sorted(pairs, key=lambda p: sorted(p)):
        if not pair <= keep or len(pair) != 2:
            continue
        a, b = sorted(pair, key=names.index)
        out.setdefault(a, []).append(b)
    return out


def merge_attached(new: dict[str, list[str]], old: dict[str, list[str]], names: list[str]) -> dict[str, list[str]]:
    """Keep every original entry that involves an attached-object link."""
    for link, others in (old or {}).items():
        for other in others or []:
            if "attached_object" not in link and "attached_object" not in other:
                continue
            if link in names and other in names and other not in new.get(link, []):
                new.setdefault(link, []).append(other)
    return new


def dump_keep_comments(path: Path, data: dict) -> None:
    original = path.read_text().splitlines()
    blocks, pending = [], []
    for line in original:
        if line.lstrip().startswith("#"):
            pending.append(line)
        elif pending:
            blocks.append((pending, line))
            pending = []
    out = yaml.safe_dump(data, sort_keys=False, default_flow_style=False, allow_unicode=True).splitlines()
    for anchor, comments in SOURCE_COMMENTS.items():
        blocks = [(c, a) for c, a in blocks if a != anchor]
        blocks.append((comments, anchor))
    for comments, anchor in blocks:
        if anchor in out:
            idx = out.index(anchor)
            out[idx:idx] = comments
    path.write_text("\n".join(out) + "\n")


def main() -> None:
    fitted = yaml.safe_load(FITTED.read_text())
    stamp = datetime.now().strftime("%H%M%S")
    backup = BACKUP_ROOT / f"before-apply-{stamp}"
    backup.mkdir(parents=True, exist_ok=True)
    for f in [*CUROBO_DIR.glob("*.yaml"), SOURCE]:
        shutil.copy2(f, backup / f.name)
    print("backup:", backup)

    for f in sorted(CUROBO_DIR.glob("*.yaml")):
        data = yaml.safe_load(f.read_text())
        kin = data["robot_cfg"]["kinematics"]
        names = [n for n in kin["collision_link_names"] if n not in UNDERCARRIAGE]
        spheres = {}
        for n in names:
            if n in fitted:
                spheres[n] = fitted[n]
            elif n in kin["collision_spheres"]:
                spheres[n] = kin["collision_spheres"][n]  # attached-object placeholders
        missing = [n for n in names if n not in spheres and not n.startswith("attached_object")]
        if missing:
            raise RuntimeError(f"{f.name}: no spheres for {missing}")
        kin["collision_link_names"] = names
        kin["collision_spheres"] = spheres
        kin["self_collision_ignore"] = merge_attached(
            rebuilt_ignore(names), kin.get("self_collision_ignore"), names
        )
        for key in ("self_collision_buffer",):
            if isinstance(kin.get(key), dict):
                kin[key] = {k: v for k, v in kin[key].items() if k in names}
        f.write_text(yaml.safe_dump(data, sort_keys=False, default_flow_style=False, allow_unicode=True))
        n_pairs = sum(len(v) for v in kin["self_collision_ignore"].values())
        print(f"{f.name}: {len(names)} collision links, {sum(len(s) for s in spheres.values())} spheres, {n_pairs} ignored pairs")

    source = yaml.safe_load(SOURCE.read_text())
    cur = source["curobo"]
    link_names = [n for n in cur["collision_spheres"] if n not in UNDERCARRIAGE]
    cur["collision_spheres"] = {n: fitted.get(n, cur["collision_spheres"][n]) for n in link_names}
    cur["self_collision_ignore"] = rebuilt_ignore(link_names)
    if isinstance(cur.get("self_collision_buffer"), dict):
        cur["self_collision_buffer"] = {k: v for k, v in cur["self_collision_buffer"].items() if k in link_names}
    dump_keep_comments(SOURCE, source)
    print(f"{SOURCE.name}: {len(link_names)} links with spheres")


if __name__ == "__main__":
    main()
