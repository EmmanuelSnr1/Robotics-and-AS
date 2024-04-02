import cv2
import numpy as np

def detect_objects(image):
    """
    Detects objects based on thresholding in the grayscale image.
    Args:
        image (np.ndarray): The image captured by the robot's camera.
    Returns:
        list of dict: A list of detected objects with their centroid and size.
    """
    # GRAY SCALE THRESHOLDING FAILED TO WORK PROPERLY SO I TRANSITIONED TO COLOUR SEGMENTATION INSTEAD.
    # Convert image from RGBA to grayscale for OpenCV processing
    gray_image = cv2.cvtColor(image , cv2.COLOR_RGBA2GRAY)
    
    # Apply a binary threshold to get a binary image
    _, thresh = cv2.threshold(gray_image, 128, 255, cv2.THRESH_BINARY)
    
    
    #COLOUR SEGMENTATION
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # These values are approximate and may need adjustment
    lower_greenish_yellow = np.array([20, 50, 50])  # Lower HSV values
    upper_greenish_yellow = np.array([40, 255, 255])  # Upper HSV values
    
    # Create a mask based on the specified range
    mask = cv2.inRange(hsv, lower_greenish_yellow, upper_greenish_yellow)
    
    # Find contours on the binary image
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter out very small contours that could be noise
    filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 50]
    
    # Debug: Print the number of contours found before filtering
    print(f"Found {len(contours)} contours before filtering.")
    
    copy = image.copy()
    
    objects = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > 24:  # Adjust this threshold as necessary
            # Draw a green rectangle around the detected object
            cv2.rectangle(copy, (x, y), (x + w, y + h), (0, 255, 0), 2)
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                rect_size = 1  # This will create a BLUE rectangle at the center of the object
                top_left = (cx - rect_size, cy - rect_size)
                bottom_right = (cx + rect_size, cy + rect_size)
                
                # Draw a blue rectangle at the centroid
                cv2.rectangle(copy, top_left, bottom_right, (255, 0, 0), -1) 
                objects.append({'centroid': (cx, cy), 'size': area})

    # Scale and display the image with detected objects
    scale = 4
    vid_name = "Detected Objects"
    cv2.namedWindow(vid_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(vid_name, image.shape[1] * scale, image.shape[0] * scale)
    cv2.imshow(vid_name, copy)  # Display the copy with rectangles
    cv2.waitKey(0)  # Wait indefinitely for a key press
    cv2.destroyAllWindows()  # Close the window after a key press

    # Debug: Print the number of objects identified
    print(f"Identified {len(objects)} objects after filtering.")

    return objects
