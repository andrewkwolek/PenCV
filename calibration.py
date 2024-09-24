import cv2
import time
from scipy.spatial.transform import Rotation
import numpy as np
import pickle

from camera import Camera
from arm import Robot


class Calibration:
    def __init__(self):
        self.vals = {}
        self.img_coords = []
        self.robot_coords = []
        self.resolution = [640, 480]
        self.fps = 30
        self.joint_pos = [[1.25, -0.5, 0, 0.5],
                          [0.75, 0.25, -0.5, 0], [0.25, 0.25, 0.25, -0.5], [0, -0.5, 1, -0.75], [-0.25, -1.25, 1, 0], [-0.25, 0.75, -0.5, -0.5], [-0.75, 0, 1.25, -1.5]]

    def set_hsv(self):
        with Camera() as cam:
            while True:
                cam.pipeline_iteration()
                key = cv2.waitKey(1)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('c'):
                    self.vals = cam.get_calibration_values()
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break

    def movement_routine(self):
        with Robot() as rob:
            with Camera(self.resolution, self.fps, self.vals["min_hue"], self.vals["depth"], self.vals["min_sat"], self.vals["min_val"], self.vals["max_hue"], self.vals["max_sat"], self.vals["max_val"]) as cam:
                rob.open()
                time.sleep(0.5)
                rob.close()
                time.sleep(2)

                # Iterate through calibration points
                for joint_position in self.joint_pos:
                    print(joint_position)
                    rob.move_all_joints(joint_position)
                    time.sleep(1)
                    self.get_coords(rob, cam)

                rob.robot.arm.go_to_home_pose()
                rob.open()
                rob.robot.arm.go_to_sleep_pose()

    def get_coords(self, rob, cam):
        time.sleep(1)
        cam_coord = cam.pipeline_iteration()
        while cam_coord == None:
            cam_coord = cam.pipeline_iteration()
        self.img_coords.append(np.array(cam_coord))
        self.robot_coords.append(rob.get_gripper_coords())

    def save_vals(self):
        with open('cal.pkl', 'wb') as file:
            pickle.dump(self.vals, file)

    def calibration_math(self):
        cam_cen = np.array([0, 0, 0])
        rob_cen = np.array([0, 0, 0])
        n = len(self.img_coords)
        for i in range(len(self.img_coords)):
            cam_cen = np.add(cam_cen, self.img_coords[i])
            rob_cen = np.add(rob_cen, self.robot_coords[i])

        cam_cen = np.divide(cam_cen, np.array([n, n, n]))
        rob_cen = np.divide(rob_cen, np.array([n, n, n]))

        print(cam_cen, rob_cen)

        normalized_cam = []
        normalized_rob = []

        for i in range(len(self.img_coords)):
            normalized_cam.append(np.subtract(self.img_coords[i], cam_cen))
            normalized_rob.append(np.subtract(self.robot_coords[i], rob_cen))

        rot, _ = Rotation.align_vectors(normalized_rob, normalized_cam)
        R = rot.as_matrix()
        P = self.img_coords[1]
        Q = self.robot_coords[1]
        RP = np.matmul(R, np.transpose(cam_cen))
        t = np.subtract(rob_cen, RP)
        self.vals["R"] = R
        self.vals["t"] = t

        test_Q = np.transpose(np.add(np.matmul(R, np.transpose(P)), t))
        print(Q, test_Q)


# main loop for calibration


def main():
    calibrate = Calibration()
    calibrate.set_hsv()
    calibrate.movement_routine()
    calibrate.calibration_math()
    calibrate.save_vals()


if __name__ == "__main__":
    main()
