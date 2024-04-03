import numpy as np
import kinpy as kp
import math
from scipy.spatial.transform import Rotation as R

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
        
    def pixel_to_base_frame(self, pixel_x, pixel_y, depth_z, intrinsic_matrix, transformation_matrix):
        """
        Transform pixel coordinates to robot base frame coordinates.
    
        Parameters:
        pixel_x (float): The x-coordinate in pixel.
        pixel_y (float): The y-coordinate in pixel.
        depth_z (float): The depth in meters from the camera frame.
        intrinsic_matrix (np.array): The camera's intrinsic matrix.
        transformation_matrix (np.array): The transformation matrix from the camera to the robot base frame.
    
        Returns:
        np.array: The (x, y, z) coordinates in the robot base frame.
        """
        # Convert pixel coordinates to normalized image coordinates
        pixel_point = np.array([pixel_x, pixel_y, 1])
        normalized_point = np.linalg.inv(intrinsic_matrix).dot(pixel_point)
    
        # Scale by the depth to get the point in the camera's 3D space
        camera_frame_point = normalized_point * depth_z
    
        # Convert the point to homogenous coordinates by adding a 1
        camera_frame_point_homogeneous = np.append(camera_frame_point, 1)
    
        # Apply the transformation matrix to get the point in the robot base frame
        base_frame_point_homogeneous = transformation_matrix.dot(camera_frame_point_homogeneous)
    
        # Convert back to non-homogeneous coordinates (drop the last element)
        base_frame_point = base_frame_point_homogeneous[:3]
    
        return base_frame_point
        
    def convert_detections_to_base_frame(self, detections, depth_z, intrinsic_matrix, transformation_matrix):
        """
        Convert a list of detections from pixel coordinates to the robot base frame coordinates.
    
        Parameters:
        detections (list of dicts): Each detection is a dictionary with centroid (x, y) and size.
        depth_z (float): The depth in meters from the camera frame.
        intrinsic_matrix (np.array): The camera's intrinsic matrix.
        transformation_matrix (np.array): The transformation matrix from the camera to the robot base frame.
    
        Returns:
        list: A list of dictionaries, each containing the object's centroid in robot base frame coordinates and size.
        """
        converted_detections = []
    
        for detection in detections:
            pixel_x, pixel_y = detection['centroid']
            size = detection['size']
            
            # Use the earlier defined pixel_to_base_frame function here
            base_frame_point = self.pixel_to_base_frame(pixel_x, pixel_y, depth_z, intrinsic_matrix, transformation_matrix)
            
            converted_detections.append({
                'centroid': base_frame_point,
                'size': size
            })
    
        return converted_detections
    
    def approach_object(self, object_centroid, approach_height=0.05):
        # Compute the target position above the object
        target_position_above_object = np.array([
            object_centroid[0], 
            object_centroid[1], 
            object_centroid[2] + approach_height
        ])
        
        # Compute the joint positions for the target position
        target_joint_pos = self.inverse_kinematics(target_position_above_object)
        
        # Move the robot to the computed joint positions
        self.move_to_joint_pos(target_joint_pos)
        
    def grasp_object(self, object_centroid):
        # Lower to the object's position
        target_joint_pos = self.inverse_kinematics(object_centroid)
        self.move_to_joint_pos(target_joint_pos)
        
        # Close the gripper to grasp the object
        self.close_gripper()
        
        # Optionally, lift the object slightly for clearance
        self.move_to_joint_pos(self.joint_pos(), target_joint_pos[2] + 0.05)

    def move_to_tray_and_release(self, tray_position):
        # Move above the tray
        target_joint_pos = self.inverse_kinematics(tray_position)
        self.move_to_joint_pos(target_joint_pos)
        
        # Open the gripper to release the object
        self.open_gripper()
        
# Class initialization and setup omitted for brevity...

if __name__ == '__main__':
    # Initialize robot and move to home position
    robot = UR5e()
    robot.move_to_joint_pos(robot.home_pos)
    # robot.close_gripper()  # Ensure gripper functions are properly implemented
    # robot.open_gripper()
    
    # Set up the camera's intrinsic matrix based on the simulation parameters
    field_of_view = 1  # Field of view in radians
    image_width = 128  # Image width in pixels
    image_height = 64  # Image height in pixels
    
    # Focal length calculation based on field of view and image dimensions
    focal_length_x = image_width / (2 * math.tan(field_of_view / 2))
    focal_length_y = image_height / (2 * math.tan(field_of_view / 2))  # Assumes square pixels
    
    # Optical center calculation
    optical_center_x = image_width / 2
    optical_center_y = image_height / 2
    
    # Construct the intrinsic matrix
    intrinsic_matrix = np.array([
        [focal_length_x, 0, optical_center_x],
        [0, focal_length_y, optical_center_y],
        [0, 0, 1]
    ])
    
    # Define the transformation matrix (assuming no rotation and a translation of 0.05m in Z)
    transformation_matrix = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0.05],
        [0, 0, 0, 1]
    ])
    
    # Set the depth value, assuming a constant height for objects on the table
    depth_z = 0.1  # Example depth value in meters
    
    # Obtain the camera image and detect objects
    img = robot.get_camera_image()
    detected_objects = detect_objects(img)
    
    # Convert the object detections to robot base frame coordinates
    base_frame_detections = robot.convert_detections_to_base_frame(detected_objects, depth_z, intrinsic_matrix, transformation_matrix)
    print(base_frame_detections)
    
    for object_info in base_frame_detections:
        centroid = object_info['centroid']
        robot.approach_object(centroid)
        robot.grasp_object(centroid)
        tray_position = np.array([0.5, 0.0, 0.2])  # Define tray position
        robot.move_to_tray_and_release(tray_position)
    # Display the camera image and detected objects
    display_image(img, 'Camera View')
    # Displaying the number of detected objects (uncomment if needed)
    # print(f"Detected {len(detected_objects)} objects.")

    
