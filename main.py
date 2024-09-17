import pyrealsense2 as rs
import numpy as np
import cv2
from camera import Camera

def main():
    resolution = [640, 480]
    fps = 30
    cam = Camera(resolution, fps)
    

if __name__ == "__main__":
    main()