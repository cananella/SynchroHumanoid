import sapien
import numpy as np
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import *
from mani_skill.sensors.camera import CameraConfig
from mani_skill.agents.registration import register_agent
import os

thard_party_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + "/third_party"

@register_agent()
class Openarm(BaseAgent):
    uid = "openarm"
    mjcf_path = thard_party_path + "/openarm_mujoco/v1/openarm_bimanual.xml"
    keyframes = dict(
        rest=Keyframe(
            pose=sapien.Pose(p=[0, 0, 0]),
            qpos=np.array(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            ),
        )
    )

    left_arm_joint_names = [
        "openarm_left_joint1",
        "openarm_left_joint2",
        "openarm_left_joint3",
        "openarm_left_joint4",
        "openarm_left_joint5",
        "openarm_left_joint6",
        "openarm_left_joint7"
    ]
    right_arm_joint_names = [
        "openarm_right_joint1",
        "openarm_right_joint2",
        "openarm_right_joint3",
        "openarm_right_joint4",
        "openarm_right_joint5",
        "openarm_right_joint6",
        "openarm_right_joint7"
    ]
    # Joint limits from MJCF: [left1, right1, left2, right2, left3, right3, left4, right4, left5, right5, left6, right6, left7, right7]
    left_arm_limit_lower = np.array([-3.490659, -3.316125, -1.570796, 0.0, -1.570796, -0.785398, -1.570796])
    left_arm_limit_upper = np.array([1.396263, 0.174533, 1.570796, 2.443461, 1.570796, 0.785398, 1.570796])
    
    right_arm_limit_lower = np.array([-1.396263, -0.174533, -1.570796, 0.0, -1.570796, -0.785398, -1.570796])
    right_arm_limit_upper = np.array([3.490659, 3.316125, 1.570796, 2.443461, 1.570796, 0.785398, 1.570796])

    gripper_left_joint_names = [
        "openarm_left_finger_joint1",
        "openarm_left_finger_joint2"
    ]
    girpper_right_joint_names = [
        "openarm_right_finger_joint1",
        "openarm_right_finger_joint2"
    ]

    arm_stiffness = 1e3
    arm_damping = 1e2
    arm_force_limit = 100

    gripper_stiffness = 1e3
    gripper_damping = 1e2
    gripper_force_limit = 100

    @property
    def _sensor_configs(self):
        head_camera = CameraConfig(
                uid="head_camera",
                pose=sapien.Pose(p=[0, 0, 0], q=[1, 0, 0, 0]),
                width=128,
                height=128,
                fov=np.pi / 2,
                near=0.01,
                far=100,
                mount=self.robot.links_map["openarm_body_link0"],
            )
        return [head_camera]

    @property
    def _controller_configs(self):
        # Joint limits from MJCF: [left1, right1, left2, right2, left3, right3, left4, right4, left5, right5, left6, right6, left7, right7]
        left_arm_pd_joint_pos = PDJointPosControllerConfig(
            self.left_arm_joint_names,
            lower=self.left_arm_limit_lower,
            upper=self.left_arm_limit_upper,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit,
            normalize_action=False,
        )
        right_arm_pd_joint_pos = PDJointPosControllerConfig(
            self.right_arm_joint_names,
            lower=self.right_arm_limit_lower,
            upper=self.right_arm_limit_upper,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit,
            normalize_action=False,
        )

        left_arm_pd_joint_delta_pos = PDJointPosControllerConfig(
            self.left_arm_joint_names,
            lower=-0.1,
            upper=0.1,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit,
            use_delta=True,
        )
        right_arm_pd_joint_delta_pos = PDJointPosControllerConfig(
            self.right_arm_joint_names,
            lower=-0.1,
            upper=0.1,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit,
            use_delta=True,
        )

        left_gripper_pd_joint_pos = PDJointPosMimicControllerConfig(
            self.gripper_left_joint_names,
            lower=-0.01,  # a trick to have force when the object is thin
            upper=0.04,
            stiffness=self.gripper_stiffness,
            damping=self.gripper_damping,
            force_limit=self.gripper_force_limit,
            mimic={
                "openarm_left_finger_joint1": {"joint" : "openarm_left_finger_joint2", "multiplier": -1.0},
            }
        )
        right_gripper_pd_joint_pos = PDJointPosMimicControllerConfig(
            self.girpper_right_joint_names,
            lower=-0.01,  # a trick to have force when the object is thin
            upper=0.04,
            stiffness=self.gripper_stiffness,
            damping=self.gripper_damping,
            force_limit=self.gripper_force_limit,
            mimic={
                "openarm_right_finger_joint1": {"joint" : "openarm_right_finger_joint2", "multiplier": -1.0},
            }
        )

        controller_configs = dict(
            pd_joint_delta_pos=dict(
                left_arm = left_arm_pd_joint_delta_pos,
                right_arm = right_arm_pd_joint_delta_pos, 
                left_gripper = left_gripper_pd_joint_pos,
                right_gripper = right_gripper_pd_joint_pos,
            ),
            pd_joint_pos=dict(
              left_arm = left_arm_pd_joint_pos, 
              right_arm = right_arm_pd_joint_pos,
              left_gripper = left_gripper_pd_joint_pos, 
              right_gripper = right_gripper_pd_joint_pos
            ),
        )

        return deepcopy_dict(controller_configs)