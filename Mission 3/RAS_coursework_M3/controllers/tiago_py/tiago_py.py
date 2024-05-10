#
# The TiaGo robot is expected to approach four of the items in the environment.
# It must do so, avoid collisions with any of the static or mobile obstacles
# (i.e., walls, other robots and the cones), and it must reach the goals
# regardless of its starting position. The possible goal items are:
#  - the wooden box with the red top: `red`
#  - the wooden box with the green top: `green`
#  - the container with the ducks: `ducks`
#  - the container with the balls: `balls`
#
# The objects will be considered "close" when the distance to the TiaGo robot
# is lower than 0.8m.
#
# The input will be given using the keyboard, typing the previously mentioned
# shortened names (e.g., red, green, ducks, balls) in the 3D view (remember
# to click inside it) and pressing enter. Goals can be provided separated by
# commas. If new goals are provided before meeting the current goals, the new
# ones must be queued (the existing ones must be satisfied before continuing).
#
# The documentation is expected to be in the code, not an external document.
#

import math
from controller import Robot
from keyboardreader import KeyboardReader
from goalchecker import get_goals_in_range
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

while robot.step(timestep) != -1:
    command = keyboard.get_command()
    if command is not None:
        print(f'Got command: {command}')
        goal_queue.extend(command.split(','))

    # Check if there are goals in the queue and if we need to generate a new plan
    if goal_queue and not current_plan:
        current_goal = goal_queue[0]
        # Modify the problem file to set the new goal (if needed)
        # planner = PDDLPlanner('domain.pddl', 'problem.pddl')  # Reinitialize planner if problem file changes
        current_plan = planner.plan()
        print(f'Generated plan: {current_plan}')

    if current_plan:
        # Execute the current step in the plan
        current_step = current_plan[0]
        if navigator.execute_plan_step(current_step):
            current_plan.pop(0)  # Remove the completed step from the plan
            if not current_plan:
                print(f'Goal {goal_queue.pop(0)} achieved.')  # Remove the achieved goal from the queue

    # Perform obstacle avoidance and general navigation
    navigator.navigate() 


