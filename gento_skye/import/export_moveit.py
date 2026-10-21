"""Export Gento Skye's collision model for MoveIt (FCL) from what cuRobo uses.

Writes into gento-skye-import/moveit/:

* ``gento_skye_spheres.urdf``: the robot URDF with every collision link's <collision> replaced by
  the cuRobo spheres (one <collision><geometry><sphere> per sphere). Same model as cuRobo.
* ``gento_skye_hulls.urdf`` + ``meshes/``: the same links with one convex-hull STL per mesh part,
  the solid the spheres were fitted to. Exact up to concavities; FCL checks convex meshes directly.
* ``gento_skye.srdf``: planning groups and <disable_collisions> from cuRobo's self-collision ignore
  list, with MoveIt Setup Assistant's reasons: Adjacent (parent/child), Always (overlapping in the
  reset pose with the spheres), Never (everything else in the list: rigid groups and pairs that
  never came within 5 cm in 10000 sampled configurations).

Links outside cuRobo's collision list keep their visuals but get no collision, as in cuRobo:
the undercarriage (wheels, forks, suspension, casters) is covered by base_link, and the sensor
links (IMU, head/eye/waist cameras) sit inside the head, torso or chassis shells. Left with their
meshes they would touch their parent link in every configuration.

Usage (from /root/gpufree-data/aspire):

    ASPIRE/.venv-behavior/bin/python gento-skye-import/export_moveit.py
"""

from __future__ import annotations

import copy
import itertools
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import apply_collision_spheres as apply  # noqa: E402
import check_collision_spheres as check  # noqa: E402
import fit_collision_spheres as fit  # noqa: E402

URDF_IN = check.DESCRIPTION / "urdf/gento_skye.urdf"
OUT = HERE / "moveit"
MESH_PACKAGE = "package://gento_skye_moveit_collision/meshes"


def load_curobo() -> dict:
    return yaml.safe_load(check.CUROBO_DEFAULT.read_text())["robot_cfg"]["kinematics"]


def strip_collisions(link: ET.Element) -> None:
    for col in link.findall("collision"):
        link.remove(col)


def add_collision(link: ET.Element, xyz, geometry: ET.Element, name: str) -> None:
    col = ET.SubElement(link, "collision", {"name": name})
    ET.SubElement(col, "origin", {"xyz": " ".join(f"{v:.5f}" for v in xyz), "rpy": "0 0 0"})
    geom = ET.SubElement(col, "geometry")
    geom.append(geometry)


def write_urdf(tree: ET.ElementTree, path: Path) -> None:
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def export_spheres(kin: dict) -> Path:
    tree = ET.parse(URDF_IN)
    root = tree.getroot()
    spheres = kin["collision_spheres"]
    names = set(kin["collision_link_names"])
    for link in root.findall("link"):
        name = link.get("name")
        strip_collisions(link)
        if name in names and spheres.get(name):
            for i, s in enumerate(spheres[name]):
                add_collision(link, s["center"], ET.Element("sphere", {"radius": f"{s['radius']:.5f}"}), f"{name}_sphere_{i}")
    path = OUT / "gento_skye_spheres.urdf"
    write_urdf(tree, path)
    return path


def export_hulls(kin: dict) -> Path:
    mesh_dir = OUT / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(URDF_IN)
    root = tree.getroot()
    names = set(kin["collision_link_names"])
    for link in root.findall("link"):
        name = link.get("name")
        if name not in names or not kin["collision_spheres"].get(name):
            strip_collisions(link)
            continue
        mesh = trimesh.load(fit.mesh_path(name), force="mesh")
        if float(mesh.bounding_box.extents.max()) < 0.005:
            # Marker link (TCP) without real geometry: use its cuRobo sphere.
            strip_collisions(link)
            for i, s in enumerate(kin["collision_spheres"][name]):
                add_collision(link, s["center"], ET.Element("sphere", {"radius": f"{s['radius']:.5f}"}), f"{name}_sphere_{i}")
            continue
        solid = fit.Solid(mesh)
        strip_collisions(link)
        for i, hull in enumerate(solid.hulls):
            fname = f"{name}_hull_{i}.stl"
            hull.export(mesh_dir / fname)
            add_collision(link, (0.0, 0.0, 0.0), ET.Element("mesh", {"filename": f"{MESH_PACKAGE}/{fname}"}), f"{name}_hull_{i}")
    path = OUT / "gento_skye_hulls.urdf"
    write_urdf(tree, path)
    return path


def export_srdf(kin: dict) -> Path:
    import yourdfpy

    urdf = yourdfpy.URDF.load(str(URDF_IN), load_meshes=False, build_scene_graph=False)
    adjacent = {frozenset((j.parent, j.child)) for j in urdf.robot.joints}
    spheres = {n: kin["collision_spheres"][n] for n in kin["collision_link_names"] if kin["collision_spheres"].get(n)}
    model = check.Model(spheres)
    world = model.world_spheres(check.reset_joint_positions())

    def overlapping(a: str, b: str) -> bool:
        if a not in world or b not in world:
            return False
        ca, ra = world[a]
        cb, rb = world[b]
        return bool(((ra[:, None] + rb[None]) - np.linalg.norm(ca[:, None] - cb[None], axis=-1)).max() > 0)

    root = ET.Element("robot", {"name": "gento_skye"})
    root.append(ET.Comment(" Planning groups: adjust in MoveIt Setup Assistant if needed. "))
    for side, eef in (("left", "left_eef_link"), ("right", "right_eef_link")):
        group = ET.SubElement(root, "group", {"name": f"{side}_arm"})
        ET.SubElement(group, "chain", {"base_link": "waist_Link_3", "tip_link": eef})
    torso = ET.SubElement(root, "group", {"name": "torso"})
    ET.SubElement(torso, "chain", {"base_link": "base_link", "tip_link": "waist_Link_3"})
    for side, finger_a, finger_b in (("left", "Arm_L10_Link", "Arm_L11_Link"), ("right", "Arm_R10_Link", "Arm_R11_Link")):
        group = ET.SubElement(root, "group", {"name": f"{side}_gripper"})
        ET.SubElement(group, "link", {"name": finger_a})
        ET.SubElement(group, "link", {"name": finger_b})
        ET.SubElement(root, "end_effector", {
            "name": f"{side}_hand", "parent_link": f"{side}_eef_link",
            "group": f"{side}_gripper", "parent_group": f"{side}_arm",
        })
    root.append(ET.Comment(" Generated from the cuRobo self_collision_ignore list (apply_collision_spheres.py). "))
    counts = {"Adjacent": 0, "Always": 0, "Never": 0}
    seen = set()
    for a, others in kin["self_collision_ignore"].items():
        for b in others:
            key = frozenset((a, b))
            if key in seen or "attached_object" in a or "attached_object" in b:
                continue
            seen.add(key)
            reason = "Adjacent" if key in adjacent else ("Always" if overlapping(a, b) else "Never")
            counts[reason] += 1
            ET.SubElement(root, "disable_collisions", {"link1": a, "link2": b, "reason": reason})
    # Undercarriage links have no collision geometry; nothing to disable for them.
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    path = OUT / "gento_skye.srdf"
    tree.write(path, encoding="utf-8", xml_declaration=True)
    print("srdf disable_collisions:", counts)
    return path


def main() -> None:
    OUT.mkdir(exist_ok=True)
    kin = load_curobo()
    for path in (export_spheres(kin), export_hulls(kin), export_srdf(kin)):
        print("wrote", path)


if __name__ == "__main__":
    main()
