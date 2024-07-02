import os
import time
import numpy as np
from third_party.DynamixelSDK.python.src.dynamixel_sdk import *  # Uses Dynamixel SDK library
from gello.robots.robot import Robot

class DynamixelActiveBot(Robot):

    def __init__(self, robot_ip: str = "127.0.0.1", robot_port: int = 6001):

        # Default setting for the Dynamixel
        self.DXL_IDs = [4, 5, 6] # Dynamixel IDs
        self.BAUDRATE = 57600  # Dynamixel default baudrate : 1000000
        self.DEVICENAME = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISI2Y-if00-port0'  # Check which port is being used on your controller

        # Control table address
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 0

        # Data Byte Length
        self.LEN_GOAL_POSITION = 4
        self.LEN_PRESENT_POSITION = 4

        # Protocol version
        self.PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel


        self.TORQUE_ENABLE = 1  # Value for enabling the torque
        self.TORQUE_DISABLE = 0  # Value for disabling the torque
        self.DXL_MINIMUM_POSITION_VALUE = 0  # Dynamixel will rotate between this value
        self.DXL_MAXIMUM_POSITION_VALUE = 4095  # and this value
        self.DXL_MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold

        self.ADDR_PRESENT_POSITION = 132
        self.LEN_PRESENT_POSITION = 4

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

    def open_port(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

    def set_baudrate(self):
        if self. portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

    def enable_torque(self, dxl_id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel %d has been successfully connected" % dxl_id)

    def sync_write_goal_position(self, goal_positions):
        groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION)
        
        for dxl_id, goal_position in zip(self.DXL_IDs, self.goal_positions):
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_position)), DXL_HIBYTE(DXL_LOWORD(goal_position)),
                                DXL_LOBYTE(DXL_HIWORD(goal_position)), DXL_HIBYTE(DXL_HIWORD(goal_position))]
            dxl_addparam_result = groupSyncWrite.addParam(dxl_id, param_goal_position)
            if not dxl_addparam_result:
                print("[ID:%03d] groupSyncWrite addparam failed" % dxl_id)
                quit()
        
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        
        groupSyncWrite.clearParam()


    def get_observations(self):
        _groupSyncRead = GroupSyncRead(
            self._portHandler,
            self._packetHandler,
            self.ADDR_PRESENT_POSITION,
            self.LEN_PRESENT_POSITION,
        )
        # Continuously read joint angles and update the joint_angles array
        while not self._stop_thread.is_set():
            time.sleep(0.001)
            with self._lock:
                _joint_angles = np.zeros(len(self.DXL_IDs), dtype=int)
                dxl_comm_result = _groupSyncRead.txRxPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"warning, comm failed: {dxl_comm_result}")
                    continue
                for i, dxl_id in enumerate(self.DXL_IDs):
                    if _groupSyncRead.isAvailable(
                        dxl_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION
                    ):
                        angle = _groupSyncRead.getData(
                            dxl_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION
                        )
                        angle = np.int32(np.uint32(angle))
                        _joint_angles[i] = angle
                    else:
                        raise RuntimeError(
                            f"Failed to get joint angles for Dynamixel with ID {dxl_id}"
                        )
                self._joint_angles = _joint_angles
            _groupSyncRead.clearParam() # TODO what does this do? should i add it
            return _joint_angles

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        return 6

    def read_present_position(self, dxl_id):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return dxl_present_position

    def map_position_to_dynamixel(position):
        # Convert degrees to Dynamixel units (0 to 4095)
        mapped = int(position * (4095 / 360))

        # if position < 0:
        dynamixel_position = 2048 + mapped        

        return dynamixel_position

    def command_joint_state(self, target_angle):
        # Start port and set baudrate
        self.open_port()
        self.set_baudrate()
        
        # Enable torque for all motors
        for dxl_id in self.DXL_IDs:
            self.enable_torque(dxl_id)

        try:

            while True:
                # Ask user for the target angle
                # target_angle = int(input("Enter target angle (-180 to 180 degrees): "))

                # Map the target angle to Dynamixel position
                goal_position = self.map_position_to_dynamixel(target_angle)
                print("Goal: ", goal_position)
                # goal_position = 0
                # Move to the target position
                self.sync_write_goal_position([goal_position] * len(self.DXL_IDs))

                time.sleep(3)

                # Read and print present position
                for dxl_id in self.DXL_IDs:
                    present_position = self.read_present_position(dxl_id)
                    print("[ID:%03d] Present Position : %d" % (dxl_id, present_position))

        except KeyboardInterrupt:
            print("Terminating program...")

        finally:
            # Disable torque for all motors before exiting
            for dxl_id in self.DXL_IDs:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            self.portHandler.closePort()

    def set_angles(self):

        # Start port and set baudrate
        self.open_port()
        self.set_baudrate()

        # Enable torque for all motors
        for dxl_id in self.DXL_IDs:
            self.enable_torque(dxl_id)

        target_angles = [0, 90, 180]

        goal_poses = [self.map_position_to_dynamixel(angle) for angle in target_angles]

        print("Goal poses: ", goal_poses)

        # Read and print present position
        for dxl_id in self.DXL_IDs:
            present_position = self.read_present_position(dxl_id)
            print("[ID:%03d] Present Position : %d" % (dxl_id, present_position))

# if __name__ == "__main__":
#     set_angles()
