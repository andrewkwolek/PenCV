import pyrealsense2 as rs
import numpy as np
import cv2
from camera import Camera

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

# Global variables for sliders 
min_hue = 0
max_hue = 180
min_sat = 0
max_sat = 255
min_val = 0
max_val = 255
depth = 1

def main():
    resolution = [640, 480]
    fps = 30
    cv2.namedWindow('image')
    cv2.createTrackbar('depth', 'image', depth, 12, change_depth)
    cv2.createTrackbar('minH', 'image', min_hue, 180, change_min_hue)
    cv2.createTrackbar('maxH', 'image', max_hue, 180, change_max_hue)
    cv2.createTrackbar('minS', 'image', min_sat, 255, change_min_saturation)
    cv2.createTrackbar('maxS', 'image', max_sat, 255, change_max_saturation)
    cv2.createTrackbar('minV', 'image', min_val, 255, change_min_hue)
    cv2.createTrackbar('maxV', 'image', max_val, 255, change_max_hue)
    with Camera(resolution, fps) as cam:
        clip_color = 153
        alpha = 0.3
        while True:
            cam.set_depth_scale(depth)
            depth_image, color_image = cam.get_images()
            bg_removed = cam.clip_image(clip_color, depth_image, color_image)
            images = cam.render_images(depth_image, bg_removed, alpha)
            cam.display_images(images)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break



if __name__ == "__main__":
    main()