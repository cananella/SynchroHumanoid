import openarm_maniskill_env
import gymnasium as gym
import time
import numpy as np
import torch
import cv2
from mani_skill.utils import common
from mani_skill.envs.sapien_env import BaseEnv
import sapien.core as sapien
import sapien.utils.viewer
from transforms3d.quaternions import axangle2quat, quat2mat
from mani_skill.agents.base_agent import BaseAgent
from mani_skill.agents.controllers.base_controller import DictController
from mani_skill.agents.controllers.base_controller import CombinedController
from mani_skill.agents.controllers.pd_joint_pos import PDJointPosController
from mani_skill.utils.structs import Articulation, ArticulationJoint, Link


def main():
    # === 환경 설정 ===
    env = gym.make(
        "Empty-v1",
        obs_mode="none",
        reward_mode="none",
        render_mode="human",
        robot_uids="openarm",
        control_mode="pd_joint_pos",
        sim_backend="auto",
        sim_config=dict(sim_freq=100, control_freq=50),
    )

    env.reset(seed=0)
    env: BaseEnv = env.unwrapped
    controller : CombinedController = env.agent.controller
    agent : BaseAgent = env.agent
    robot = agent.robot 
    # viewer = env.render_human()

    left_arm_controller = controller.controllers['left_arm']
    right_arm_controller = controller.controllers['right_arm']
    left_gripper_controller = controller.controllers['left_gripper']
    right_gripper_controller = controller.controllers['right_gripper']
    print(left_arm_controller.qpos.cpu())
    print(right_arm_controller.qpos.cpu())
    print(left_gripper_controller.qpos.cpu())
    print(right_gripper_controller.qpos.cpu())
    print(env.action_space.shape)
    

    while True:

        env.render()
    env.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()