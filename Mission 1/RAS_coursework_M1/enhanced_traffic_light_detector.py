class EnhancedTrafficLightDetector(TrafficLightDetector):
    def detect_traffic_light_state_within_boxes(self, hsv_image, boxes):
        detected_states = []
        for box in boxes:
            x, y, w, h = box
            # Crop the HSV image to the box's region
            cropped_hsv = hsv_image[y:y+h, x:x+w]

            # Apply color segmentation within the cropped image
            mask_red = self.get_red_mask(cropped_hsv)
            mask_green = self.get_green_mask(cropped_hsv)

            # Simple state determination based on the masks
            state = "none"
            if cv2.countNonZero(mask_red) > 50:  # Threshold for red detection, adjust as necessary
                state = "red"
            elif cv2.countNonZero(mask_green) > 50:  # Threshold for green detection, adjust as necessary
                state = "green"

            detected_states.append((box, state))

            if self.debug:
                print(f"Box at {box} detected as {state}")

        return detected_states

    def get_red_mask(self, hsv_image):
        lower_red, upper_red = self.red_range[0], self.red_range[1]
        mask_red1 = cv2.inRange(hsv_image, lower_red, upper_red)

        lower_red2, upper_red2 = (170, 120, 70), (180, 255, 255)
        mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

        return mask_red1 + mask_red2

    def get_green_mask(self, hsv_image):
        lower_green, upper_green = self.green_range
        return cv2.inRange(hsv_image, lower_green, upper_green)
