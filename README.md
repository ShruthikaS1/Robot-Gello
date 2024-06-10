# GELLO
This is the central repo that holds the all the software for GELLO. See the website for the paper and other resources for GELLO https://wuphilipp.github.io/gello_site/
See the GELLO hardware repo for the STL files and hardware instructions for building your own GELLO https://github.com/wuphilipp/gello_mechanical
```
git clone https://github.com/wuphilipp/gello_software.git
cd gello_software
```

### Instructions

1. Download and Clone the repository
2. Manually install the libraries and dependencies from the requriements.txt file or from the github link above
   ```
   cd /Assistive_Feeding_Gello
   pip3 install -e .
   ```
3. Simulation
   
   UR3e - Spoon

   ```
   python3 experiments/launch_nodes.py --robot sim_urspoon
   ```

   UR3e - Hande
   ```
   python3 experiments/launch_nodes.py --robot sim_urhande
   ```

4. Connect the Gello hardware and to Control the Robot

   UR3e Spoon
   ```
   python3 experiments/run_env_ur.py --agent=gello
   ```

   UR3e Hande
   ```
   python3 experiments/run_env.py --agent=gello
   ```
5. Additionally, joint angles can also be passed from the csv file, Run the python script below to execute one of the joint angle trajectories/to reach home position
   ```
   python3 experiments/joint_angles.py
   ```

## Switching between Spoon Mount and Gripper
Swtich to **Gripper**
1. In _gello/agents/gello_agent.py_ change the gripper_config to a tuple
2. In _experiments/run_env.py_ change the reset_joints to 7 dimensions

Switch to **Spoon mount**
1. In _gello/agents/gello_agent.py_ change the gripper_config to **None**
2. In _experiments/run_env.py_ change the reset_joints to 6 dimensions

### Utilities - Gello Hardware
To check U2D2 gello port
   ````
   ls /dev/serial/by-id
   ````
To Calibrate the Gello
   ```
   python3 scripts/gello_get_offset.py \
      --start-joints -1.57 -1.57 -1.57 -1.57 1.57 1.57 \
      --joint-signs 1 1 -1 1 1 1 \
      --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISZZM-if00-port0

   ```