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

    def set_wheel_speeds(self, left_speed, right_speed):
        self.l_motor.setVelocity(left_speed)
        self.r_motor.setVelocity(right_speed)

    def move_forward(self, speed=2):
        self.set_wheel_speeds(speed, speed)

    def rotate(self, speed=1):
        self.set_wheel_speeds(speed, -speed)

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
                self.rotate()
            else:
                self.rotating = False

    def execute_plan_step(self, step):
        # Dummy implementation to execute a plan step
        # This should include logic to navigate to the target location
        print(f'Executing step: {step}')
        # Here, you can add navigation logic to move to the waypoint or perform the action
        return True  # Return True when the step is complete
