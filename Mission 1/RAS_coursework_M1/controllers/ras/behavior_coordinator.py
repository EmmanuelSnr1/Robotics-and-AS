

class BehaviourCoordinator:
    def __init__(self):
        self.robot = robot

        self.state = "Driving"
    
    def update_state(self, traffic_light_state):
        if self.state == "Driving" and traffic_light_state == "red":
            self.state = "StoppingAtRed"
        elif self.state == "StoppingAtRed":
            self.state = "WaitingAtRed"
        elif self.state == "WaitingAtRed" and traffic_light_state == "green":
            self.state = "ProceedingAfterGreen"
        elif self.state == "ProceedingAfterGreen":
            self.state = "Driving"
    
    def execute_current_state_actions(self, vehicle_controller):
        if self.state == "Driving":
            self.robot.set_speed(45)  # Example speed, adjust as needed
            # Optionally adjust steering angle for lane following
        elif self.state == "StoppingAtRed":
            self.robot.set_speed(0)  # Stop the vehicle
        elif self.state == "WaitingAtRed":
            # Keep speed at 0 or implement wait logic
            self.robot.set_speed(0)
        elif self.state == "ProceedingAfterGreen":
            self.robot.set_speed(45)  # Resume driving
            # Adjust steering angle to continue following the lane
    
    def run(self, vehicle_controller, traffic_light_state):
        self.update_state(traffic_light_state)
        self.execute_current_state_actions()

# Example usage within the main control loop
# Assume vehicle_controller is an instance of a class controlling the vehicle's actions (e.g., set_speed, set_steering_angle)
# traffic_light_state is obtained from the TrafficLightDetector

# behaviour_coordinator = BehaviourCoordinator()

# This would be inside your main loop where `vehicle_controller` controls the car,
# and `traffic_light_state` is the current state of traffic lights detected by the TrafficLightDetector.
# behaviour_coordinator.run(vehicle_controller, traffic_light_state)
