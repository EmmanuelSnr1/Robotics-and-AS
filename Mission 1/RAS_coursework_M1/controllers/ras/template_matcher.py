import cv2
import numpy as np

class TemplateMatcher:
    def __init__(self, template_path, debug=False):
        # Load the template image during initialization
        self.template = cv2.imread(template_path, 0)  # Load template in grayscale
        self.debug = debug

    def match_template(self, image):
        # Convert the input image to grayscale
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Perform template matching
        res = cv2.matchTemplate(image_gray, self.template, cv2.TM_CCOEFF_NORMED)
        
        # Set a threshold for detecting the template
        threshold = 0.8
        
        # If the max correlation value is greater than the threshold, consider the template detected
        if np.max(res) >= threshold:
            if self.debug:
                print("Yes, detected")
            
            # If debugging is enabled, show the matched area on the image
            if self.debug:
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                top_left = max_loc
                h, w = self.template.shape
                bottom_right = (top_left[0] + w, top_left[1] + h)
                cv2.rectangle(image, top_left, bottom_right, (0, 0, 255), 2)
                cv2.imshow("Matched Image", image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            
            return True
        else:
            if self.debug:
                print("No, not detected")
            return False
