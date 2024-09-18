import pyrealsense2 as rs
import numpy as np
import cv2
from camera import Camera

# Global variables for sliders 
min_hue = 112 #104
max_hue = 135 #164
min_sat = 53 #41
max_sat = 224 #220
min_val = 23 #113
max_val = 209 #165
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
        clip_color = 0
        alpha = 0.3
        while True:
            cam.set_depth_scale(depth)
            depth_image, color_image = cam.get_images()
            # clipped_image = cam.clip_image(clip_color, depth_image, color_image)
            # rgb_aligned_image = cam.render_images(depth_image, bg_removed, alpha)
            filtered_image = cam.bilateral_filter(color_image, d, sig_col, sig_space)
            hsv_aligned_image, mask = cam.rgb_to_hsv(filtered_image, (min_hue, min_sat, min_val), (max_hue, max_sat, max_val))
            contours = cam.contours(color_image, mask)
            cam.locate_centroid(contours, color_image)
            images = cam.render_images(color_image, hsv_aligned_image)
            cam.display_images(images)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break



if __name__ == "__main__":
    main()