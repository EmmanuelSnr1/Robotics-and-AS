import cv2
from rasrobot import RASRobot
from lane_controller_simple import LaneController



class MyRobot(RASRobot):
    def __init__(self):
        """
        The constructor has no parameters.
        """
        super(MyRobot, self).__init__()

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
    
            # use lane controller to determine steering angle and speed
            steering_angle, speed = lc.predict_angle_and_speed(image)

            # set steering angle and speed of car (will be applied in next tick)
            self.set_steering_angle(steering_angle)
            self.set_speed(speed)


if __name__ == '__main__':
    # create a robot and let it run!
    robot = MyRobot()
    robot.run()
