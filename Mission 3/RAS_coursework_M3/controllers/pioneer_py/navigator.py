import time
import math
from controller import Robot

class Navigator:
    def __init__(self, robot, obstacle_threshold=0.4):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        self.lidar = robot.getDevice("lidar")
        self.lidar.enable(self.timestep)
        self.gps = robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = robot.getDevice("compass")
        self.compass.enable(self.timestep)

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

    def navigate_to(self, target_x, target_y):
        current_x, current_y = self.gps.getValues()[0], self.gps.getValues()[1]
        distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
        
        # Simplified navigation logic
        if distance > 0.1:  # Threshold to determine if the robot is close enough to the target
            self.move_forward()
            return False  # Not yet at the target
        else:
            self.set_wheel_speeds(0, 0)  # Stop the robot
            return True  # Reached the target

    def execute_plan_step(self, step):
        # Parse the plan step
        tokens = step.strip('()').split()
        action = tokens[0]
        args = tokens[1:]

        # Handle actions
        if action == 'move':
            robot_id, from_location, to_location = args
            # Get target coordinates for the destination (to_location)
            target_coords = self.get_location_coordinates(to_location)
            if target_coords:
                target_x, target_y = target_coords
                print(f'Moving to {to_location} at coordinates ({target_x}, {target_y})')
                return self.navigate_to(target_x, target_y)
        elif action == 'pick_up':
            robot_id, item, location = args
            print(f'Picking up {item} at {location}')
            # Simulate picking up item (implement actual logic if needed)
            time.sleep(1)
            return True
        elif action == 'drop_off':
            robot_id, item, location = args
            print(f'Dropping off {item} at {location}')
            # Simulate dropping off item (implement actual logic if needed)
            time.sleep(1)
            return True
        else:
            print(f'Unknown action: {action}')
            return True

    def get_location_coordinates(self, location):
        # Dummy function to return coordinates for locations
        locations = {
            'l1': (0, 0),
            'l2': (1, 0),
            'l3': (1, 1),
            'l4': (0, 1),
        }
        return locations.get(location, None)

# Example usage:
# robot = Robot()
# navigator = Navigator(robot)
# step = "(move r1 l1 l2)"
# navigator.execute_plan_step(step)
