import os

import cv2
import numpy as np


class FireDetector:
    def __init__(self) -> None:
        pass
    
    def locate_fire(self, img):
        '''
        Locate fire in image.

        return pixel coordinates of fire bounding box center if detected; None otherwise.
        '''

        firing_trees = self.__get_firing_trees(img) 
        
        if not self.__has_fire(firing_trees):
            self.__show_result(img, fire=False)
            return None

        fire_pixels = np.where(firing_trees != 0)
        
        min_y = np.min(fire_pixels[0])
        max_y = np.max(fire_pixels[0])
        min_x = np.min(fire_pixels[1])
        max_x = np.max(fire_pixels[1])

        start = (min_x, max_y)
        end = (max_x, min_y)
        self.__show_result(img, fire=True, start=start, end=end)

        mid_x = int((min_x + max_x)/2)
        mid_y = int((min_y + max_y)/2)
        
        return [mid_x, mid_y]

    def __show_result(self, img, fire=True, start=None, end=None):
        if fire:
            img = cv2.rectangle(img, start, end, color=(255, 0, 0), thickness=4)
        else:
            color = (0,255,0)
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_size = 1
            font_color = color
            font_thickness = 2
            text = 'NO FIRE DETECTED'
            x,y = 10, 50
            img = cv2.putText(img, text, (x,y), font, font_size, font_color, font_thickness, cv2.LINE_AA)
        
        cv2.imshow('result', img)
        cv2.waitKey(10)

    def __get_firing_trees(self, img):
        img_trees = self.__remove_ground(img)
        _, green, red = cv2.split(img_trees)

        ret, red_mask = cv2.threshold(red, 140, 255, cv2.THRESH_BINARY)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel=np.ones((5, 5), np.uint8))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel=np.ones((5, 5), np.uint8))
        
        ret, green_mask = cv2.threshold(green, 100, 255, cv2.THRESH_BINARY)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel=np.ones((5, 5), np.uint8))
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel=np.ones((5, 5), np.uint8))
        green_mask = cv2.bitwise_not(green_mask)

        final_mask = cv2.bitwise_and(red_mask, green_mask)

        result = cv2.bitwise_and(img_trees, img_trees, mask=final_mask)

        # cv2.imshow('red', red)
        # cv2.imshow('green', green)
        # cv2.imshow('red mask', red_mask)
        # cv2.imshow('green mask', green_mask)
        # cv2.imshow('original', img)
        # cv2.imshow('trees', result)
        # cv2.waitKey(3000)

        return result

    def __has_fire(self, img):
        ''' Returns True if the image contains the fire. False otherwise. '''
        if np.sum(img) > 100000:
            return True
        else:
            return False

    def __remove_ground(self, img):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        kernel=np.ones((3, 3), np.uint8)
        
        s = img_hsv[:,:,1]
        s = np.where(s < 127, 0, 1) 
    
        v = (img_hsv[:,:,2] + 127) % 255
        v = np.where(v > 127, 1, 0)

        foreground = np.where(s+v > 0, 1, 0).astype(np.uint8)
        foreground = cv2.bitwise_and(img, img, mask=foreground)

        fore_gray = cv2.cvtColor(foreground, cv2.COLOR_BGR2GRAY)
        fore_mask = cv2.morphologyEx(fore_gray, cv2.MORPH_OPEN, kernel)

        finalimage = cv2.bitwise_and(foreground, foreground, mask=fore_mask)

        return finalimage


if __name__ == '__main__':
    parent = os.path.dirname(os.getcwd())
    img_dir = os.path.join(parent, 'images')

    fire_detector = FireDetector()
    for i in range(1, 2, 1):
        img_name = 'img{}.png'.format(i)
        img = cv2.imread('{}/{}'.format(img_dir, img_name))
        fire_location = fire_detector.locate_fire(img)

        print('2D location is: {}'.format(tuple(fire_location)))

