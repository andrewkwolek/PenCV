import pyrealsense2 as rs
import numpy as np
import cv2


class Camera:
    def __init__(self, resolution=[640, 480], fps=30, min_hue=110, depth=6, min_sat=129, min_val=43, max_hue=139, max_sat=255, max_val=255, d=5, sig_col=75, sig_space=75):
        print("init")
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        self.config.enable_stream(
            rs.stream.depth, resolution[0], resolution[1], rs.format.z16, fps)
        self.config.enable_stream(
            rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, fps)

        # Set default HSV values
        self.min_hue = min_hue
        self.max_hue = max_hue
        self.min_sat = min_sat
        self.max_sat = max_sat
        self.min_val = min_val
        self.max_val = max_val
        self.depth = depth

        # Set default bilateral filter values
        self.d = d
        self.sig_col = sig_col
        self.sig_space = sig_space

    def __enter__(self):
        print("enter")
        cv2.namedWindow('image')
        cv2.createTrackbar('depth', 'image', self.depth, 12, self.change_depth)
        cv2.createTrackbar('minH', 'image', self.min_hue,
                           180, self.change_min_hue)
        cv2.createTrackbar('maxH', 'image', self.max_hue,
                           180, self.change_max_hue)
        cv2.createTrackbar('minS', 'image', self.min_sat,
                           255, self.change_min_saturation)
        cv2.createTrackbar('maxS', 'image', self.max_sat,
                           255, self.change_max_saturation)
        cv2.createTrackbar('minV', 'image', self.min_val,
                           255, self.change_min_value)
        cv2.createTrackbar('maxV', 'image', self.max_val,
                           255, self.change_max_value)
        cv2.createTrackbar('d', 'image', self.d, 9, self.change_d)
        cv2.createTrackbar('sigColor', 'image', self.sig_col,
                           150, self.change_sigma_color)
        cv2.createTrackbar('sigSpace', 'image', self.sig_space,
                           150, self.change_sigma_space)
        self.profile = self.pipeline.start(self.config)
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.pipeline.stop()

    def change_depth(self, val):
        self.depth = val

    def change_min_hue(self, val):
        self.min_hue = val

    def change_max_hue(self, val):
        self.max_hue = val

    def change_min_saturation(self, val):
        self.min_sat = val

    def change_max_saturation(self, val):
        self.max_sat = val

    def change_min_value(self, val):
        self.min_val = val

    def change_max_value(self, val):
        self.max_val = val

    def change_d(self, val):
        if self.d < 1:
            pass
        else:
            self.d = val

    def change_sigma_color(self, val):
        self.sig_col = val

    def change_sigma_space(self, val):
        self.sig_space = val

    def set_depth_scale(self, clip_dist_in_m):
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        # print("Depth Scale is: ", self.depth_scale)

        # We will be removing the background of objects more than
        #  clipping_distance_in_meters meters away
        self.clipping_distance = clip_dist_in_m / self.depth_scale

    def get_images(self):
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        align = rs.align(align_to)
        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        # aligned_depth_frame is a 640x480 depth image
        self.aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        # print(self.aligned_depth_frame.get_units())

        depth_image = np.asanyarray(self.aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return depth_image, color_image

    def clip_image(self, clip_color, depth_image, color_image):
        # Remove background - Set pixels further than clipping_distance to grey
        # depth image is 1 channel, color is 3 channels
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
        return np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), clip_color, color_image)

    def bilateral_filter(self, img):
        return cv2.bilateralFilter(img, self.d, self.sig_col, self.sig_space)

    def rgb_to_hsv(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (self.min_hue, self.min_sat, self.min_val),
                           (self.max_hue, self.max_sat, self.max_val))
        res = cv2.bitwise_and(frame, frame, mask=mask)
        return res, mask

    def contours(self, display_image, mask):
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(display_image, contours, -1, (0, 255, 0), 1)
        return contours

    ############################ Start_Citation [1] ####################################
    def locate_centroid(self, contours, display_image):
        largest_cnt = None
        largest_area = 0
        for cnt in contours:
            cur_area = cv2.contourArea(cnt)
            if cur_area > largest_area:
                largest_area = cur_area
                largest_cnt = cnt
        M = cv2.moments(largest_cnt)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(display_image, (cx, cy), 6, (0, 0, 255), -2)
            return (cx, cy)
        else:
            return None
    ############################# End_Citation [1] ####################################

    def get_full_coordinate(self, centroid):
        if centroid[0] > 0 and centroid[0] < 480 and centroid[1] > 0 and centroid[1] < 640:
            im_depth = self.aligned_depth_frame.get_distance(
                centroid[0], centroid[1])
            print(centroid)
            print(im_depth)
        else:
            im_depth = 0
        return (centroid[0], centroid[1], im_depth)

    def render_images(self, image1, image2):
        # Render images:
        #   depth align to color on left
        #   depth on right
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=alpha), cv2.COLORMAP_JET)
        images = np.hstack((image1, image2))
        return images

    def display_images(self, images):
        cv2.imshow('image', images)

    def pipeline_iteration(self):
        self.set_depth_scale(self.depth/6)
        depth_image, color_image = self.get_images()
        filtered_image = self.bilateral_filter(color_image)
        # clipped_image = self.clip_image(179, depth_image, filtered_image)
        # hsv_aligned_image, mask = self.rgb_to_hsv(clipped_image)
        hsv_aligned_image, mask = self.rgb_to_hsv(filtered_image)
        contours = self.contours(color_image, mask)
        centroid = self.locate_centroid(contours, color_image)
        xyz = None
        if centroid != None:
            xyz = self.get_full_coordinate(centroid)
        images = self.render_images(color_image, hsv_aligned_image)
        self.display_images(images)
        if xyz != None and xyz[2] > 0:
            coord_to_robot = xyz
            return coord_to_robot

    def get_calibration_values(self):
        return {
            "depth": self.depth,
            "min_hue": self.min_hue,
            "min_sat": self.min_sat,
            "min_val": self.min_val,
            "max_hue": self.max_hue,
            "max_sat": self.max_sat,
            "max_val": self.max_val,
            "d": self.d,
            "sig_col": self.sig_col,
            "sig_space": self.sig_space
        }


def main():
    with Camera() as cam:
        while True:
            cam.pipeline_iteration()
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('c'):
                print(cam.get_calibration_values())
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break


if __name__ == "__main__":
    main()

# [1] "Drawing the centroid for the contour of maximum area in OpenCV python", stackoverflow, 2022, https://stackoverflow.com/questions/65097816/drawing-the-centroid-for-the-contour-of-maximum-area-in-opencv-python
