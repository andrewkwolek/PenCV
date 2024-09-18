import pyrealsense2 as rs
import numpy as np
import cv2

class Camera:
    def __init__(self, resolution, fps):
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

        self.config.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, fps)

    def __enter__(self):
        print("enter")
        self.profile = self.pipeline.start(self.config)
        return self
    
    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.pipeline.stop()

    def set_depth_scale(self, clip_dist_in_m):
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: " , depth_scale)

        # We will be removing the background of objects more than
        #  clipping_distance_in_meters meters away
        self.clipping_distance = clip_dist_in_m / depth_scale

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
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return depth_image, color_image
    
    def clip_image(self, clip_color, depth_image, color_image):
        # Remove background - Set pixels further than clipping_distance to grey
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        return np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), clip_color, color_image)
    
    def bilateral_filter(self, img, d, sigCol, sigSpace):
        return cv2.bilateralFilter(img, d, sigCol, sigSpace)
    
    def rgb_to_hsv(self, frame, min_hsv, max_hsv):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, min_hsv, max_hsv)
        res = cv2.bitwise_and(frame, frame, mask=mask)
        return res, mask
    
    def contours(self, display_image, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(display_image, contours, -1, (0, 255, 0), 1)
        return contours
    
    def locate_centroid(self, contours, display_image):
        largest_cnt = None
        largest_area = 0
        for cnt in contours:
            cur_area = cv2.contourArea(cnt)
            if cur_area > largest_area:
                largest_area = cur_area
                largest_cnt = cnt
        M = cv2.moments(largest_cnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(display_image, (cx, cy), 6, (0, 0, 255), -2)

    
    def render_images(self, image1, image2):
        # Render images:
        #   depth align to color on left
        #   depth on right
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=alpha), cv2.COLORMAP_JET)
        images = np.hstack((image1, image2))
        return images
    
    def display_images(self, images):
        cv2.imshow('image', images)
