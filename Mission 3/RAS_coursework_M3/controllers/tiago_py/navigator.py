import math
import time
import random
from controller import Robot

class Navigator:
    def __init__(self, robot, obstacle_threshold=0.4):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        self.lidar = robot.getDevice("lidar")
        self.lidar.enable(self.timestep)
        self.lidar_width = self.lidar.getHorizontalResolution()
        self.obstacle_threshold = obstacle_threshold

        self.l_motor = robot.getDevice("wheel_left_joint")
        self.r_motor = robot.getDevice("wheel_right_joint")
        self.l_motor.setPosition(float('inf'))
        self.r_motor.setPosition(float('inf'))

        self.rotating = False
        self.rotating_timestamp = None
        self.rotating_time = None

        self.target_location = None
        self.locations = {
            'room1': (-2.8171, -4.17743, 0),  # Room 1: Metal storage box with balls
            'room2': (-3.08, 3.63, 0),        # Room 2: Metal storage box with ducks
            'room3': (3.68, -3.95, 0.3),      # Room 3: Green wooden box
            'room4': (3.68, 3.02, 0.3)        # Room 4: Red wooden box
        }

    def set_wheel_speeds(self, left_speed, right_speed):
        self.l_motor.setVelocity(left_speed)
        self.r_motor.setVelocity(right_speed)

    def move_forward(self, speed=5):  # Increased speed
        self.set_wheel_speeds(speed, speed)

    def rotate_towards_target(self, speed=1):
        if self.target_location:
            target_coords = self.locations.get(self.target_location)
            if target_coords:
                gps_values = self.robot.getDevice('gps').getValues()
                angle_to_target = math.atan2(target_coords[1] - gps_values[1], target_coords[0] - gps_values[0])
                compass_values = self.robot.getDevice('compass').getValues()
                current_angle = math.atan2(compass_values[0], -compass_values[2])
                angle_error = angle_to_target - current_angle

                if abs(angle_error) > 0.1:  # Adjust this threshold as needed
                    self.set_wheel_speeds(speed if angle_error < 0 else -speed, -speed if angle_error < 0 else speed)
                else:
                    self.rotating = False
                    self.set_wheel_speeds(0, 0)
            else:
                self.set_wheel_speeds(speed, -speed)  # Default random rotation if no target location
        else:
            self.set_wheel_speeds(speed, -speed)  # Default random rotation if no target location

    def avoid_obstacles(self):
        lms291_values = self.lidar.getRangeImage()
        for d in lms291_values[len(lms291_values) // 3: -len(lms291_values) // 3]:
            if d < self.obstacle_threshold:
                self.rotating = True
                self.rotating_timestamp = time.time()
                self.rotating_time = random.randint(3, 8)
                break

    def navigate(self):
        if not self.rotating:
            self.avoid_obstacles()
            if not self.rotating:
                self.move_forward()
        if self.rotating:
            if time.time() < self.rotating_timestamp + self.rotating_time:
                self.rotate_towards_target()
            else:
                self.rotating = False

    def execute_plan_step(self, step):
        print(f'Executing step: {step}')
        step = step.split('\n')[0]  # Remove additional information
        parts = step.replace('(', '').replace(')', '').split()
        if len(parts) >= 4 and parts[0] == 'movenorth':
            _, robot, from_location, to_location = parts[:4]
            print(f'Moving from {from_location} to {to_location}')
            self.target_location = to_location
            return True  # Return True when the step is accepted for execution
        return False

    def is_at_location(self, location, threshold=0.8):
        target_coords = self.locations.get(location, None)
        if target_coords is None:
            return False
        gps_values = self.robot.getDevice('gps').getValues()
        distance = ((gps_values[0] - target_coords[0]) ** 2 + (gps_values[1] - target_coords[1]) ** 2) ** 0.5
        return distance < threshold  # Check if within the specified threshold
