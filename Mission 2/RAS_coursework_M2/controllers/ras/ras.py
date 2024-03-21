import numpy as np
import kinpy as kp
from scipy.spatial.transform import Rotation as R

from util import display_image
from rasrobot import RASRobot, TIME_STEP


"""
MISSION 2: Control of a Robot Manipulator Arm for a Pick and Place Task
Learning outcomes: 1, 2, 3

Scenario:
A manipulator arm is equipped with a camera at its end-effector. 
The existing controller already provides behaviours to move
the robot in joint and/or task space and to open/close the gripper.
There are five cubes randomly distributed in the robot's work space,
and there is a crate, which is always in the same spot.

Task: 
Use the camera to detect the cubes. Implement a controller for the 
manipulator arm that clears the objects from the table and drops 
them into the crate.

Hints:
1) INSTALLATION
The project requires the kinpy library, which you should install into
your environment. It is an open source library, so you can directly 
look at the code on github.

2) TRANSFORMS
You can move the robot in task space by computing the inverse kinematics.
The kinpy.Transform object describes the pose in task space. 
See more details here: 
https://github.com/neka-nat/kinpy/blob/master/kinpy/transform.py
It consists of a pos (position as [x, y, z]) and rot (rotation). 
The rotation is a quaternion in the [w, x, y, z] format. You can include
other libraries to convert between rotation representations, such as 
rotation matrices, axis-angle, and Euler angles. Just be aware that some
there is an alternative convention for the order of elements in a quaternion,
which is [x, y, z, w] - it is a common source of errors.
scipy.spatial.transform could be a good option, but it uses the [x, y, z, w]
convention, so you would need to convert the quaternions.

3) GRASPING
Simulating contact-rich tasks is very difficult. You might notice that
the cubes sometimes act unexpectedly when being grasped.
It is generally a good idea to align the gripper as best as possible to the 
parallel surfaces of the cube, and to only move with low velocity when
picking up an object.
"""

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
        self.chain = kp.build_serial_chain_from_urdf(open(URDF_FN), end_link)
        
        # print chain on console
        print('kinematic chain:')
        print(self.chain)
        print(f'The end link of the chain is <{end_link}>.')
        print('All computations of forward and inverse kinematics apply to this link.')
        
        
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
        
        
if __name__ == '__main__':
    # initialise robot and move to home position
    robot = UR5e()
    robot.move_to_joint_pos(robot.home_pos)
    robot.close_gripper()
    robot.open_gripper()
    
    # display the camera image
    img = robot.get_camera_image()
    display_image(img, 'camera view', wait=True)  # waits for key input
    
    # todo: find cubes, grasp them and drop them into the basket!
