"""OmniGibson environment configuration for the R1 Pro pickup tasks.

Mirrors OmniGibson's ``configs/r1pro_primitives.yaml`` (absolute-position joint controllers
everywhere, which ``robot.q_to_action`` requires) with the BEHAVIOR task section the ASPIRE
reference used: presampled robot poses, no online object sampling, assisted grasping. Kept as
Python so no package data is needed.
"""

from __future__ import annotations

from copy import deepcopy

from cap_harness.behavior.registry import BehaviorTaskMetadata

ROBOT_NAME = "robot_r1"
ACTION_FREQUENCY_HZ = 30
PHYSICS_FREQUENCY_HZ = 120
HEAD_HORIZONTAL_APERTURE = 40.0
"""Wider head field of view, the value the official evaluator applies (eval/evaluator.py)."""

# 29-DoF reset for gento_skye, in the runtime articulation order printed by
# robot.joints: 6 virtual base joints, 3 waist, then head / left arm / right arm
# interleaved, followed by four finger joints. Fingers start at 0 (open); 0.04 m is closed.
RESET_JOINT_POSITIONS: tuple[float, ...] = (
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.5, 0.0, 0.0,
    0.0,                    # Head_J1_Joint
    1.57, -1.57,            # Joint1_L, Joint1_R
    0.0,                    # Head_J2_Joint
    -1.13, -1.13,           # Joint2_L, Joint2_R
    -1.18, 1.18,            # Joint3_L, Joint3_R
    -1.57, -1.57,           # Joint4_L, Joint4_R
    -0.423, 0.423,          # Joint5_L, Joint5_R
    -0.147, -0.147,         # Joint6_L, Joint6_R
    0.298, -0.298,          # Joint7_L, Joint7_R
    0.0, 0.0, 0.0, 0.0,
)

_POSITION_JOINT_CONTROLLER = {
    "name": "JointController",
    "motor_type": "position",
    "command_input_limits": None,
    "use_delta_commands": False,
    "use_impedances": False,
}


def build_environment_config(
    metadata: BehaviorTaskMetadata,
    *,
    camera_width: int,
    camera_height: int,
    horizon: int,
) -> dict[str, object]:
    """Return the ``og.Environment`` config for one task at the requested camera resolution.

    ``horizon`` sets the simulator's own step timeout one step past the harness limit so the
    run recorder, not the simulator, is the authority on ``max_steps``.
    """
    if camera_width <= 0 or camera_height <= 0 or horizon <= 0:
        raise ValueError("camera dimensions and horizon must be positive")
    return {
        "env": {
            "action_frequency": ACTION_FREQUENCY_HZ,
            "rendering_frequency": ACTION_FREQUENCY_HZ,
            "physics_frequency": PHYSICS_FREQUENCY_HZ,
            "device": None,
            "automatic_reset": False,
            "flatten_action_space": False,
            "flatten_obs_space": False,
            "external_sensors": None,
        },
        "render": {"viewer_width": 1280, "viewer_height": 720},
        "scene": {
            "type": "InteractiveTraversableScene",
            "scene_model": metadata.scene_model,
            "trav_map_resolution": 0.1,
            "default_erosion_radius": 0.0,
            "trav_map_with_objects": True,
            "num_waypoints": 1,
            "waypoint_resolution": 0.2,
            "load_object_categories": None,
            "not_load_object_categories": None,
            "load_room_types": list(metadata.load_room_types),
            "load_room_instances": None,
            "load_task_relevant_only": False,
            "seg_map_resolution": 1.0,
            "scene_source": "OG",
            "include_robots": False,
        },
        "robots": [
            {
                "model": metadata.robots[0],
                "name": ROBOT_NAME,
                "obs_modalities": ["rgb", "depth_linear", "proprio"],
                "include_sensor_names": None,
                "exclude_sensor_names": None,
                "scale": 1.0,
                "self_collisions": True,
                "action_normalize": False,
                "action_type": "continuous",
                "grasping_mode": "assisted",
                # Gento's finger joints close as q rises from 0 to 0.04 m.
                "grasping_direction": "upper",
                "reset_joint_pos": list(RESET_JOINT_POSITIONS),
                "sensor_config": {
                    "VisionSensor": {
                        "sensor_kwargs": {
                            "image_height": int(camera_height),
                            "image_width": int(camera_width),
                        }
                    }
                },
                "controller_config": {
                    "base": {
                        "name": "HolonomicBaseJointController",
                        "motor_type": "position",
                        "command_input_limits": None,
                        "use_impedances": False,
                    },
                    "trunk": deepcopy(_POSITION_JOINT_CONTROLLER),
                    # The robot default is a delta camera controller. q_to_action() can
                    # only hold a complete joint target when every group is absolute.
                    "camera": deepcopy(_POSITION_JOINT_CONTROLLER),
                    "arm_left": deepcopy(_POSITION_JOINT_CONTROLLER),
                    "arm_right": deepcopy(_POSITION_JOINT_CONTROLLER),
                    "gripper_left": deepcopy(_POSITION_JOINT_CONTROLLER),
                    "gripper_right": deepcopy(_POSITION_JOINT_CONTROLLER),
                },
            }
        ],
        "objects": [],
        "task": {
            "type": "BehaviorTask",
            "activity_name": metadata.activity_name,
            "activity_definition_id": 0,
            "activity_instance_id": 0,
            "online_object_sampling": False,
            # The challenge snapshots only contain poses for officially supported robots.
            # BehaviorAdapter loads the selected instance and applies Gento's converted pose itself.
            # Enabling this here makes Environment.__init__ reset before the adapter can inject that pose.
            "use_presampled_robot_pose": False,
            "termination_config": {"max_steps": int(horizon) + 1},
            "reward_config": {"r_potential": 1.0},
            "include_obs": False,
        },
    }


__all__ = [
    "ACTION_FREQUENCY_HZ",
    "HEAD_HORIZONTAL_APERTURE",
    "PHYSICS_FREQUENCY_HZ",
    "RESET_JOINT_POSITIONS",
    "ROBOT_NAME",
    "build_environment_config",
]
