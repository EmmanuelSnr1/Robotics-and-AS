
import numpy as np

from controller import Robot
from vehicle import Driver

import cv2

class RASRobot(object):
    def __init__(self):
        self.__robot = Driver()
        self.__timestep = int(self.__robot.getBasicTimeStep())

        self.__camera = self.__robot.getDevice("camera")
        self.__camera.enable(self.__timestep)

    def get_camera_image(self):
        
        return np.frombuffer(self.__camera.getImage(), np.uint8).reshape((64,128,4))

    def set_steering_angle(self, angle):
        
        self.__robot.setSteeringAngle(angle)

    def set_speed(self, speed):
     
        self.__robot.setCruisingSpeed(speed)
    
    def tick(self):
        
        if self.__robot.step() == -1:
            return False

        return True
        
