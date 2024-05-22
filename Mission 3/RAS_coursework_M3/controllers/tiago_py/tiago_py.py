import math
from controller import Robot
from keyboardreader import KeyboardReader
from planner import PDDLPlanner
from navigator import Navigator

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize devices
l_motor = robot.getDevice("wheel_left_joint")
r_motor = robot.getDevice("wheel_right_joint")
lidar = robot.getDevice("lidar")
lidar.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)
l_motor.setPosition(math.inf)
r_motor.setPosition(math.inf)
l_motor.setVelocity(0)
r_motor.setVelocity(0)

# Initialize modules
planner = PDDLPlanner('domain.pddl', 'problem.pddl')
keyboard = KeyboardReader(timestep)
navigator = Navigator(robot)

goal_queue = []
current_plan = []
current_action = None
behaviour = None

# Define room boundaries based on central points of the boxes
ROOM_BOUNDARIES = {
    'room1': {'min_x': -4.0, 'max_x': -1.5, 'min_y': -5.5, 'max_y': -2.5},
    'room4': {'min_x': -4.0, 'max_x': -1.5, 'min_y': 2.5, 'max_y': 5.0},
    'room3': {'min_x': 1.5, 'max_x': 4.0, 'min_y': -5.5, 'max_y': -2.5},
    'room2': {'min_x': 1.5, 'max_x': 4.0, 'min_y': 2.5, 'max_y': 5.0}
}

def determine_current_room(gps_values):
    x, y, z = gps_values
    for room, bounds in ROOM_BOUNDARIES.items():
        if bounds['min_x'] <= x <= bounds['max_x'] and bounds['min_y'] <= y <= bounds['max_y']:
            return room
    return "unknown"

def write_problem_file(current_goal):
    with open('problem.pddl', 'w') as f:
        f.write(f"""(define (problem go-to) 
  (:domain example)
  (:objects
    bot - robot
    room1 room2 room3 room4 - room
  )
  (:init
    (in bot room1) ;; Starting room should be dynamically set if needed
    ;; Connections between rooms based on the layout in the image
    (cangonorth room1 room4)
    (cangonorth room2 room3)
    (cangoeast room1 room3)
    (cangoeast room2 room4)
    (cangosouth room4 room1)
    (cangosouth room3 room2)
    (cangowest room3 room1)
    (cangowest room4 room2)
  )
  (:goal
    (in bot {current_goal})
  )
)""")

# Map input commands to room names
room_mapping = {
    'r1': 'room1',
    'r2': 'room2',
    'r3': 'room3',
    'r4': 'room4'
}

class Chill:
    def __init__(self):
        self.tick = 0
        self.state = "rotate"

    def step(self, lidar_values, compass_values):
        l_motor.setVelocity(2)  # Endless rotation
        r_motor.setVelocity(-2)  # Endless rotation

while robot.step(timestep) != -1:
    command = keyboard.get_command()
    if command is not None:
        print(f'Got command: {command}')
        goal_queue.extend(command.split(','))

    # Check if there are goals in the queue and if we need to generate a new plan
    if goal_queue and not current_plan:
        current_goal = goal_queue[0]
        if current_goal in room_mapping:
            # Write the problem file with the new goal
            write_problem_file(room_mapping[current_goal])
            current_plan = planner.plan()
            print(f'Generated plan: {current_plan}')
        else:
            print(f'Invalid goal: {current_goal}')
            goal_queue.pop(0)  # Remove invalid goal

    if current_plan:
        # Execute the current step in the plan
        current_step = current_plan[0]
        if navigator.execute_plan_step(current_step):
            current_plan.pop(0)  # Remove the completed step from the plan
            if not current_plan:
                print(f'Goal {goal_queue.pop(0)} achieved.')  # Remove the achieved goal from the queue
                behaviour = Chill()  # Set to Chill once goal is achieved

    # Perform obstacle avoidance and general navigation
    navigator.navigate()

    # Check if the robot has reached the target location
    if navigator.target_location and navigator.is_at_location(navigator.target_location, threshold=0.1): #This thresholding is currently setting to reach at a reasonable distance but i will leave it like this. 
        print(f'Reached location: {navigator.target_location}')
        behaviour = Chill()  # Transition to Chill state
        navigator.target_location = None  # Clear the target location
        current_plan.clear()  # Clear the current plan

    # Update behaviour state
    if behaviour:
        lidar_values = lidar.getRangeImage()[1:-1]
        compass_values = compass.getValues()
        behaviour.step(lidar_values, compass_values)

    # Determine current room and print the state
    current_room = determine_current_room(gps.getValues())
    print(f"Current Room: {current_room}, Current State: {behaviour.state if behaviour else 'navigating'}")
