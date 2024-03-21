import numpy as np
import cv2


def display_image(image, name, scale=2, wait=False):
    """ 
    function to display an image 
    :param image: ndarray, the image to display
    :param name: string, a name for the window
    :param scale: int, optional, scaling factor for the image
    :param wait: bool, optional, if True, will wait for click/button to close window
    """
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, image.shape[1]*scale, image.shape[0]*scale)
    cv2.imshow(name, image)
    cv2.waitKey(0 if wait else 1)
    
    