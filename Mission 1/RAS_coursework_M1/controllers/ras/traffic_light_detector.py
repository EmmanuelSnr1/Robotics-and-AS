import cv2
import numpy as np

class TrafficLightDetector:
    def __init__(self, debug=False):
        # Define HSV color ranges for red and green
        self.red_range = ((0, 120, 70), (10, 255, 255))
        self.green_range = ((40, 40, 40), (90, 255, 255))
        self.debug = debug  # Add a debug flag

    def preprocess_image(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return hsv_image

    def detect_traffic_light(self, hsv_image):
        # Apply color segmentation and detection logic
        # Define HSV range for red color
        lower_red, upper_red = self.red_range[0], self.red_range[1]
        mask_red1 = cv2.inRange(hsv_image, lower_red, upper_red)
        
        # Define another HSV range for red (due to HSV's circular nature)
        lower_red2, upper_red2 = (170, 120, 70), (180, 255, 255)
        mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        
        # Combine both red masks
        mask_red = mask_red1 + mask_red2
    
        # Define HSV range for green color
        lower_green, upper_green = self.green_range
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
    
        # Detect red contours
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Detect green contours
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Debugging outputs
        if self.debug:
            print(f"Red contours detected: {len(contours_red)}")
            print(f"Green contours detected: {len(contours_green)}")
        
        state = "none"  # Default to 'none' if no significant detections
        # Determine state based on detected contours
        if len(contours_red) > 0:
            state = "red"
            if self.debug:
                print("Detected color: Red")
        if len(contours_green) > 0:
            state = "green"
            if self.debug:
                print("Detected color: Green")
        
        # Additional logic can be implemented to handle cases where both colors are detected
        
        return state
