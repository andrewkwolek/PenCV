import pyrealsense2 as rs
import numpy as np
import cv2
from camera import Camera

def main():
    resolution = [640, 480]
    fps = 30
    with Camera(resolution, fps) as cam:
        print(cam)
        cam.set_depth_scale(0.25)
        while True:
            depth, color = cam.get_images()
            bg_removed = cam.clip_image(153, depth, color)
            images = cam.render_images(depth, bg_removed)
            cam.display_images(images)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break



if __name__ == "__main__":
    main()