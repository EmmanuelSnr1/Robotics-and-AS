import cv2
from rasrobot import RASRobot
from lane_controller_simple import LaneController
from traffic_light_detector import TrafficLightDetector  # Import the TrafficLightDetector class

class MyRobot(RASRobot):
    def __init__(self):
        """
        The constructor has no parameters.
        """
        super(MyRobot, self).__init__()
        self.traffic_light_detector = TrafficLightDetector(debug=True)  # Initialize the traffic light detector

    def run(self):
        """
        This function implements the main loop of the robot.
        """
        # initialise lane controller
        lc = LaneController(debug=False)  # set debug to True if you want visualisation and logs

        while self.tick():
            # get front camera from car and display it
            image = self.get_camera_image()
            name = 'Snrs Camera image'
            scale = 2
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(name, image.shape[1]*scale, image.shape[0]*scale)
            cv2.imshow(name, image)
            cv2.waitKey(1)

            # Detect traffic light color
            hsv_image = self.traffic_light_detector.preprocess_image(image)
            traffic_light_state = self.traffic_light_detector.detect_traffic_light(hsv_image)
            if traffic_light_state == 'red':
                # If red light is detected, stop the car
                speed = 0
                steering_angle = 0  # Keep steering angle unchanged or set to desired value
            else:
                # use lane controller to determine steering angle and speed in normal conditions
                steering_angle, speed = lc.predict_angle_and_speed(image)

            # set steering angle and speed of car (will be applied in next tick)
            self.set_steering_angle(steering_angle)
            self.set_speed(speed)

if __name__ == '__main__':
    # create a robot and let it run!
    robot = MyRobot()
    robot.run()
