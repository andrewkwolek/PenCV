import pyrealsense2 as rs
import numpy as np
import cv2
from camera import Camera
from arm import Robot

# Global variables for sliders 
min_hue = 112 #104
max_hue = 135 #164
min_sat = 53 #41
max_sat = 224 #220
min_val = 23 #0
max_val = 209 #242
depth = 6
d = 5
sig_col = 75
sig_space = 75

def change_depth(val):
    global depth
    depth = val / 6

def change_min_hue(val):
    global min_hue
    min_hue = val

def change_max_hue(val):
    global max_hue
    max_hue = val

def change_min_saturation(val):
    global min_sat 
    min_sat = val

def change_max_saturation(val):
    global max_sat 
    max_sat = val

def change_min_value(val):
    global min_val 
    min_val = val

def change_max_value(val):
    global max_val 
    max_val = val

def change_d(val):
    global d
    if d < 1:
        pass
    else:
        d = val

def change_sigma_color(val):
    global sig_col
    sig_col = val

def change_sigma_space(val):
    global sig_space
    sig_space = val

def main():
    resolution = [640, 480]
    fps = 30
    coord_to_robot = None
    cv2.namedWindow('image')
    cv2.createTrackbar('depth', 'image', depth, 12, change_depth)
    cv2.createTrackbar('minH', 'image', min_hue, 180, change_min_hue)
    cv2.createTrackbar('maxH', 'image', max_hue, 180, change_max_hue)
    cv2.createTrackbar('minS', 'image', min_sat, 255, change_min_saturation)
    cv2.createTrackbar('maxS', 'image', max_sat, 255, change_max_saturation)
    cv2.createTrackbar('minV', 'image', min_val, 255, change_min_value)
    cv2.createTrackbar('maxV', 'image', max_val, 255, change_max_value)
    cv2.createTrackbar('d', 'image', d, 9, change_d)
    cv2.createTrackbar('sigColor', 'image', sig_col, 150, change_sigma_color)
    cv2.createTrackbar('sigSpace', 'image', sig_space, 150, change_sigma_space)
    with Camera(resolution, fps) as cam:
        while True:
            cam.pipeline_iteration(depth, d, sig_col, sig_space, min_hue, min_sat, min_val, max_hue, max_sat, max_val)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
            
    with Robot() as arm:    
        arm.run()



if __name__ == "__main__":
    main()