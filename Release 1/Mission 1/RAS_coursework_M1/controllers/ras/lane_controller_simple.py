import numpy as np
import cv2


class LaneController:
    def __init__(self, debug=False):
        # state of lane controller
        self.turn_counter = 0
        self.modes = ['driving', 'turning_left', 'turning_right', 'stop_for_red_light']
        self.mode = 'driving'  # Initial mode
        
        # some (static) parameters of the controller
        self.steering_row = 47
        self.crossroad_row = 45

        self.max_steering_angle = 0.3
        self.max_speed = 45
        self.min_speed = 25

        # flag for debugging
        self.debug = debug

    def _display_image(self, image, name, scale=2):
        """
        if debug mode is active, show the image
        """
        if not self.debug:
            return

        image = image.copy()
        image[self.steering_row, :] = 1.0 - image[self.steering_row, :]  # mark steering row
        image[self.crossroad_row, :] = 1.0 - image[self.crossroad_row, :]  # mark crossroad row

        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, image.shape[1]*scale, image.shape[0]*scale)
        cv2.imshow(name, image)
        cv2.waitKey(1)

    def _get_road(self, image):
        """
        This method detects the road. It takes the image, converts it into HSV (Hue, Saturation, Value) and
        filters pixels that are likely to be part of the road.
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower = np.array([0, 0, 0], np.uint8)
        upper = np.array([200, 50, 100], np.uint8)
        mask_tarmac = cv2.inRange(hsv, lower, upper).astype(bool)

        lower = np.array([25, 100, 100], np.uint8)
        upper = np.array([50, 255, 255], np.uint8)
        mask_yellow_road_markings = cv2.inRange(hsv, lower, upper).astype(bool)

        binary_image = np.zeros(image.shape)
        binary_image[mask_tarmac] = 1.0
        binary_image[mask_yellow_road_markings] = 1.0
        binary_image = binary_image[:, :, 0]  # reduce to 1 channel (gray image)
        reduce_noise = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, (3, 3))
        self._display_image(reduce_noise, 'road image')
        return reduce_noise

    def _get_road_center(self, road_image):
        """
        This method uses the binarised road image to determine the center of the road.
        It is returned in the range [-1, 1], where -1 means the center is on the left and +1 on the right.
        If the road is not visible, it returns None.
        """
        road_img_line = road_image[self.steering_row, :]
        if road_img_line.sum() == 0:
            # no road visible
            return None

        idx = road_img_line.nonzero()[0]
        cx = np.mean(idx / len(road_img_line))  # center in [0, 1]
        center_normalised = (cx - 0.5) * 2  # normalised to [-1, 1]
        return center_normalised

    def _at_crossroads(self, road_image):
        """
        This method uses the binarised road image to determine the presence of crossroads.
        It returns True if crossroads are detected, False otherwise.
        """
        steering_sum = road_image[self.steering_row, :].sum()
        crossroad_sum = road_image[self.crossroad_row, :].sum()
        # simple detector: if there is a lot more road visible further away, we assume it's a crossroad
        diff = crossroad_sum - steering_sum
        if diff > 10 and crossroad_sum > 120:
            return True
        return False

    def _get_steering_angle(self, road_center):
        """
        this method calculates a steering angle based on the center of the road.
        """
        if road_center is None:
            return self.steering_angle
            
        # try and keep road center on the left
        preferred_road_center_pos = -0.15
        angle = road_center - preferred_road_center_pos
        
        angle = np.clip(angle, -self.max_steering_angle, self.max_steering_angle)
        return angle

    def _get_speed(self, steering_angle):
        """
        this method identifies a speed based on the steering angle.
        if the steering angle is high, we reduce the speed to avoid tipping over.
        """
        # speed factor is 1 if going straight and 0 if turning maximally
        speed_factor = (self.max_steering_angle - abs(steering_angle)) / self.max_steering_angle
        # linearly interpolate between min and max speed
        speed = (self.max_speed - self.min_speed) * speed_factor + self.min_speed
        return speed

    def predict_angle_and_speed(self, image, traffic_light_detection):
        """
        this is the main function of the lane controller.
        it takes an image and returns a steering angle and a speed.
        """
        steering_angle = 0
        
        # Add logic for stopping at a red light and resuming at green
        if traffic_light_state == 'red' and self.mode != 'stop_for_red_light':
            self.mode = 'stop_for_red_light'
        elif traffic_light_state == 'green' and self.mode == 'stop_for_red_light':
            self.mode = 'driving'  # Resume driving when the light turns green
        
        if self.mode == 'stop_for_red_light':
            # Stop the vehicle by setting speed to 0
            return 0, 0 # Steering angle 0, speed 0 to stop
        elif self.mode == 'turning_left':
            # MODE 'turning_left': make a (blind) turn at a crossroads based on time passed
            self.turn_counter += 1
            if self.turn_counter < 45:
                # keep going straight first to ensure we're on the crossroads
                steering_angle = 0
            elif self.turn_counter < 120:
                # actually make the turn
                steering_angle = -self.max_steering_angle
            else:
                # we finished the turn, go back to normal driving
                self.mode = 'driving'
                self.turn_counter = 0
                if self.debug:
                    print('finished turn, back to normal driving!')
                    
        elif self.mode == 'turning_right':
            # MODE 'turning_right': make a (blind) turn at a crossroads based on time passed
            self.turn_counter += 1
            if self.turn_counter < 30:
                # keep going straight first to ensure we're on the crossroads
                steering_angle = 0
            elif self.turn_counter < 100:
                # actually make the turn
                steering_angle = self.max_steering_angle
            else:
                # we finished the turn, go back to normal driving
                self.mode = 'driving'
                self.turn_counter = 0
                if self.debug:
                    print('finished turn, back to normal driving!')

        elif self.mode == 'driving':
            # MODE 'driving': normal driving, reactive controller based on road image
            road_image = self._get_road(image)
            road_center = self._get_road_center(road_image)

            if road_center is None:
                # oh-oh, we are probably somewhere off-road... hopefully this does not happen
                if self.debug:
                    print('oh-oh, we are probably somewhere off-road... ')
                steering_angle = 0  # maybe we get back to the road?
            else:
                # normal steering
                steering_angle = self._get_steering_angle(road_center)

            # check if we need to make a turn
            if self._at_crossroads(road_image):
                # at crossroads -> change to MODE turning
                self.mode = np.random.choice(['turning_left', 'turning_right'])

                if self.debug:
                    print(f'crossroads detected! {self.mode}')

        else:
            # unknown mode, if this happens we made a programming mistake
            raise ValueError(f'unknown mode: {self.mode}')

        # in both modes, we smoothen the steering angle and adjust the speed accordingly
        speed = self._get_speed(steering_angle)

        return steering_angle, speed
