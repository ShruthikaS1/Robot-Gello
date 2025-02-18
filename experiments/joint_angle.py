import datetime
import glob
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import tyro

from gello.agents.agent import BimanualAgent, DummyAgent
from gello.agents.gello_agent import GelloAgent
from gello.data_utils.format_obs import save_frame
from gello.env import RobotEnv
from gello.robots.robot import PrintRobot
from gello.zmq_core.robot_node import ZMQClientRobot

import pandas as pd

import glob
import os
from tqdm import tqdm
import csv

def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)


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

def print_state(env):

    # while True:
    obs = env.get_obs()["joint_positions"]
    return obs
        # time.sleep(1)
        # print("Observation: ", obs)
        # moved = env.step(np.array([0, -1.57, 0, -1.57, 0, 0]))


def execute_trajectory(env):
    # data = pd.read_csv(csv_file_path)
    # Convert angles to lists of lists
    # joint_angles = data[['shoulder_pan_angle', 'shoulder_lift_angle','elbow_angle', 'wrist1_angle', 'wrist2_angle', 'wrist3_angle']].values.tolist()
    # print(joint_angles)
    joint_angles = []
    # joint_angles.append([0.0, -1.57, 0.0, -1.57, 0.0, 0.0, 1])
    joint_angles.append([-1.57, -1.57, -1.57, -1.57, 1.57, 0, 1])
    for angles in joint_angles:
        # Set the joint angles
        time.sleep(0.8)
        moved = env.step(np.array(angles))
        print(moved["joint_positions"])
        

def execute_all_csvs(env, csv_folder):
    for n, csv_file in enumerate(glob.glob(os.path.join(csv_folder, "*.csv"))):
        # Read CSV file
        dataset = pd.read_csv(csv_file, usecols=range(0, 7), engine="python").astype(np.float64)

        print(f"Executing CSV File: {n}")
        for i in tqdm(range(0, len(dataset))):
        # for i in range(0, len(dataset)):
            joint_angle = np.array(dataset.iloc[i]) 
            time.sleep(0.1)
            obs = env.step(joint_angle)
            # print(obs["ee_pos_quat"])


# def extract_ee_pos_quat(env, csv_folder):
#     for n, csv_file in enumerate(glob.glob(os.path.join(csv_folder, "*.csv"))):
#         # Read CSV file
#         dataset = pd.read_csv(csv_file, usecols=range(0, 7), engine="python").astype(np.float64)

#         print(f"Executing CSV File: {n}")
#         for i in tqdm(range(0, len(dataset))):
#         # for i in range(0, len(dataset)):
#             joint_angle = np.array(dataset.iloc[i]) 
#             time.sleep(0.1)
#             obs = env.step(joint_angle)
#             print(obs["ee_pos_quat"])
#             # write to csv file
#             with open('ee_pos_quat.csv', 'a') as f:
#                 writer = csv.writer(f)
#                 if f.tell() == 0:
#                     f.write("ee_pos_quat")
#                 f.write(f"{obs['ee_pos_quat']}\n")

if __name__ == "__main__":
    robot_client = ZMQClientRobot(port=Args.robot_port, host=Args.hostname)
    camera_clients = {
        # you can optionally add camera nodes here for imitation learning purposes
        # "wrist": ZMQClientCamera(port=args.wrist_camera_port, host=args.hostname),
        # "base": ZMQClientCamera(port=args.base_camera_port, host=args.hostname),
    }
    csv_folder =  Path(__file__).parent.parent / "csv" / "90close"

    env = RobotEnv(robot_client, control_rate_hz=Args.hz, camera_dict=camera_clients)
    # execute_trajectory(env)
    execute_all_csvs(env, csv_folder)





    

    