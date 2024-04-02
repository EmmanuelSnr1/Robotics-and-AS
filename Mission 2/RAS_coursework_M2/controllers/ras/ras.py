import numpy as np
import kinpy as kp
from scipy.spatial.transform import Rotation as R
import time


from util import display_image
from rasrobot import RASRobot, TIME_STEP
from object_detection import detect_objects


class UR5e(RASRobot):
    def __init__(self):
        """
        This is your main robot class. It inherits from RASRobot for access
        to devices such as motors, position sensors, and camera.
        """
        super().__init__()
        
        # load the kinematic chain based on the robot's URDF file
        end_link = 'wrist_3_link'  # link used for forward and inverse kinematics
        URDF_FN = '../../resources/ur5e_2f85_camera.urdf'
        # self.chain = kp.build_serial_chain_from_urdf(open(URDF_FN), end_link)
        with open(URDF_FN, 'r') as file:
            urdf_string = file.read()
        self.chain = kp.build_serial_chain_from_urdf(urdf_string, end_link)
        
        # print chain on console
        # print('kinematic chain:')
        # print(self.chain)
        # print(f'The end link of the chain is <{end_link}>.')
        # print('All computations of forward and inverse kinematics apply to this link.')
        
        
    @property
    def home_pos(self):
        """ 
        this is the home configuration of the robot 
        """
        return [1.57, -1.57, 1.57, -1.57, -1.57, 0.0]

    def joint_pos(self):
        """
        :return: ndarray, the current joint position of the robot
        """
        joint_pos = np.asarray([m.getPositionSensor().getValue() for m in self.motors])
        return joint_pos
        
    def move_to_joint_pos(self, target_joint_pos, timeout=5, velocity=0.8):
        """
        blocking behaviour, moves the robot to the desired joint position.
        :param target_joint_pos: list/ndarray with joint configuration
        :param timeout: float, timeout in seconds after which this function returns latest
        :param velocity: float, target joint velocity in radians/second
        :return: bool, True if robot reaches the target position
                  else will return False after timeout (in seconds)
        """
        if len(target_joint_pos) != len(self.motors):
            raise ValueError('target joint configuration has unexpected length')
            
        for pos, motor in zip(target_joint_pos, self.motors):
            motor.setPosition(pos)
            if velocity is not None:
                motor.setVelocity(velocity)
            
        # step through simulation until timeout or position reache
        for step in range(int(timeout * 1000) // TIME_STEP):
            self.step()

            # check if the robot is close enough to the target position
            if all(abs(target_joint_pos - self.joint_pos()) < 0.001):
                return True
                
        print('Timeout. Robot has not reached the desired target position.')
        return False
        
    def forward_kinematics(self, joint_pos=None):
        """
        computes the pose of the chain's end link for given joint position.
        :param joint_pos: joint position for which to compute the end-effector pose
                          if None given, will use the robot's current joint position
        :return: kinpy.Transform object with pos and rot
        """
        if joint_pos is None:
            joint_pos = self.joint_pos()
            
        ee_pose = self.chain.forward_kinematics(joint_pos)
        return ee_pose
        
    def inverse_kinematics(self, target_pose):
        """
        Computes a joint configuration to reach the given target pose.
        Note that the resulting joint position might not actually reach the target
        if the target is e.g. too far away.
        :param target_pose: kinpy.Transform, pose of the end link of the chain
        :return: list/ndarray, joint position
        """
        ik_result = self.chain.inverse_kinematics(target_pose, self.joint_pos())
        return ik_result
        
    def hover_between_poses(self, poses, iterations=10):
        """
        Makes the robot arm hover between two poses.

        :param poses: A list of two kinpy.Transform objects representing target poses.
        :param iterations: How many times to move back and forth.
        """
        for _ in range(iterations):
            for pose in poses:
                # Compute the inverse kinematics to get the joint positions for the target pose.
                joint_positions = self.inverse_kinematics(pose)

                # Move the robot arm to the computed joint positions.
                success = self.move_to_joint_pos(joint_positions)
                if not success:
                    print("Movement to target position failed or timed out")

                # Wait a bit before moving to the next position to simulate hovering.
                time.sleep(1)
    
        
        
if __name__ == '__main__':
    # initialise robot and move to home position
    robot = UR5e()
    # robot.move_to_joint_pos(robot.home_pos)
    # robot.close_gripper()
    # robot.open_gripper()
    pose1 = [0.4, 0.0, 0.2]
    pose2 = [0.4, 0.3, 0.4]
    robot.hover_between_poses([pose1, pose2], iterations=10)
    
    
    # display the camera image
    img = robot.get_camera_image()
    objects = detect_objects(img)
    print (objects)
    # print(f"Detected {len(objects)} objects.")
    display_image(img, 'camera view', wait=True)  # waits for key input
    
