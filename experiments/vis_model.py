import numpy as np
from copy import deepcopy


import robomimic
import robomimic.utils.file_utils as FileUtils
import robomimic.utils.torch_utils as TorchUtils
import robomimic.utils.tensor_utils as TensorUtils
import robomimic.utils.obs_utils as ObsUtils
from robomimic.envs.env_base import EnvBase
from robomimic.algo import RolloutPolicy
from dataclasses import dataclass
from typing import Optional, Tuple
from gello.env import RobotEnv

from gello.zmq_core.robot_node import ZMQClientRobot

@dataclass
class Args:
    agent: str = "none"
    robot_port: int = 6001
    # robot_port: int = 50003  # for trajectory
    wrist_camera_port: int = 5000
    base_camera_port: int = 5001
    hostname: str = "127.0.0.1"

    #---- Hardware ---
    # hostname: str = "192.168.77.243"
    # robot_ip: str = "192.168.77.21"
    robot_type: str = None  # only needed for quest agent or spacemouse agent
    hz: int = 100
    start_joints: Optional[Tuple[float, ...]] = None

    gello_port: Optional[str] = None
    mock: bool = False
    use_save_interface: bool = False
    data_dir: str = "~/bc_data"
    bimanual: bool = False
    verbose: bool = False



# set the model path
# model_path = "/mnt/f/gcodes/RLHF-gello_software/robomimic/bc_trained_models_default/test/20240609111832/models/model_epoch_2000.pth"
model_path = "/mnt/f/gcodes/RLHF-gello_software/robomimic/bc_trained_models/test/20240609110611/model_epoch_50.pth"

# set the device
device = TorchUtils.get_torch_device(try_to_use_cuda=True)
print(f"Device being used currently is: {device}")

# Restore policy
policy, ckpt_dict = FileUtils.policy_from_checkpoint(ckpt_path=model_path, device = device)

# create environment from saved model
# env, _ = FileUtils.env_from_checkpoint(ckpt_dict=ckpt_dict, render=False, render_offscreen=True, verbose=True)
# obs = env.reset()
# print("Observations: ", obs)
# act = policy(ob=obs["robot0_joint_obs"])
# print(obs["robot0_joint_pos"])
# print("Action: ", act)


# # ## ------ Start Gello Environment ----- ##
robot_client = ZMQClientRobot(port = Args.robot_port, host = Args.hostname)

camera_clients = {

}

env = RobotEnv(robot_client, control_rate_hz = Args.hz, camera_dict = camera_clients)

print("Gello Environment")
# print(env.get_obs()["joint_positions"])


print("Visualizing Model: ")
gello_obs = env.get_obs()
for i in range(0, 1000):
    obs_to_pass = {"robot0_joint_pos": gello_obs["joint_positions"]}
    # obs_to_pass["robot0_joint_pos"] = np.append(obs_to_pass["robot0_joint_pos"], -1)        # Default model 
    act = policy(obs_to_pass)
    # act = act[:-1]                                                                          # Default model
    print("Action: ", act)
    gello_obs = env.step(act)


