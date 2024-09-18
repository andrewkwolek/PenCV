from camera import Camera
from arm import Robot
import cv2

# Global variables for sliders 
min_hue = 112 #104
max_hue = 135 #164
min_sat = 53 #41
max_sat = 224 #220
min_val = 23 #0
max_val = 209 
depth = 6
d = 5
sig_col = 75
sig_space = 75

def main():
    iter = 0
    while iter < 5:
        resolution = [640, 480]
        fps = 30
        coord_to_robot = None
        with Camera(resolution, fps) as cam:
                while True:
                    cam.set_depth_scale(depth)
                    depth_image, color_image = cam.get_images()
                    # clipped_image = cam.clip_image(clip_color, depth_image, color_image)
                    # rgb_aligned_image = cam.render_images(depth_image, bg_removed, alpha)
                    filtered_image = cam.bilateral_filter(color_image, d, sig_col, sig_space)
                    hsv_aligned_image, mask = cam.rgb_to_hsv(filtered_image, (min_hue, min_sat, min_val), (max_hue, max_sat, max_val))
                    contours = cam.contours(color_image, mask)
                    centroid = cam.locate_centroid(contours, color_image)
                    xyz = None
                    if centroid != None:
                        xyz = cam.get_full_coordinate(depth_image, centroid)
                    images = cam.render_images(color_image, hsv_aligned_image)
                    cam.display_images(images)
                    if xyz != None and xyz[2] > 0:
                        coord_to_robot = xyz
                        print(coord_to_robot)

                    key = cv2.waitKey(1)
                    # Press esc or 'q' to close the image window
                    if key & 0xFF == ord('q') or key == 27:
                        cv2.destroyAllWindows()
                        break
        
        with Robot() as arm:
            arm.calibration_test(coord_to_robot)


if __name__ == "__main__":
    main()