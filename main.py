import numpy as np
import cv2
import time
import pickle

from camera import Camera
from arm import Robot


def main():
    resolution = [640, 480]
    fps = 30
    vals = {}
    with open('cal.pkl', 'rb') as file:
        vals = pickle.load(file)
        print(vals)

    with Camera(resolution, fps, vals["min_hue"], vals["depth"], vals["min_sat"], vals["min_val"], vals["max_hue"], vals["max_sat"], vals["max_val"]) as cam:
        while True:
            P = np.array(cam.pipeline_iteration())
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
        with Robot() as rob:
            R = vals["R"]
            t = vals["t"]

            rob.robot.arm.go_to_home_pose(moving_time=1)
            time.sleep(1)
            rob.robot.gripper.set_pressure(0.75)
            rob.robot.gripper.release()
            Q = rob.find_translation(R, P, t)
            print(Q)

            rob.robot.arm.set_ee_pose_components(
                Q[0], Q[1], Q[2]-0.1, moving_time=3)
            rob.robot.arm.set_ee_pose_components(
                Q[0], Q[1], Q[2]+0.15, moving_time=1)

            time.sleep(0.5)
            rob.robot.gripper.grasp()
            time.sleep(2)
            rob.robot.arm.go_to_home_pose()
            time.sleep(1)
            rob.robot.arm.go_to_sleep_pose()


if __name__ == "__main__":
    main()
