from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
import numpy as np
import time
import cv2

from camera import Camera
# The robot object is what you use to control the robot


class Robot:
    def __init__(self):
        self.robot = InterbotixManipulatorXS("px100", "arm", "gripper")

    def __enter__(self):
        robot_startup()
        self.mode = 'h'
        self.robot.arm.go_to_home_pose(moving_time=1)
        self.robot.gripper.set_pressure(0.8)
        return self

    def __exit__(self,  exc_type, exc_value, exc_traceback):
        robot_shutdown()
        time.sleep(1)

    def get_mode(self):
        return self.mode

    def get_gripper_coords(self):
        M = self.robot.arm.get_ee_pose()
        d = np.array([M[0][3], M[1][3], M[2][3]])
        print(d)
        return d

    def get_joint_angles(self):
        print(f"Joint angles {self.robot.arm.get_joint_commands()}")

    def find_translation(self, R, P, t):
        RP = np.matmul(R, np.transpose(P))
        Q = np.transpose(np.add(RP, np.transpose(t)))
        return Q

    # Changes the angle of joint by delta rads
    # Returns true if completed
    def move_joint(self, joint: str, delta: int):
        pos = self.robot.arm.get_single_joint_command(joint)
        # time.sleep(0.25)
        return self.robot.arm.set_single_joint_position(joint, pos + delta, moving_time=0.5)

    def move_all_joints(self, joints):
        self.robot.arm.set_joint_positions(joints, moving_time=3)

    def close(self):
        self.robot.gripper.grasp()

    def open(self):
        self.robot.gripper.release()

    def run_robot_poses(self):
        while self.mode != 'q':
            self.mode = input(
                "[h]ome, [s]leep, [q]uit, [o]pen, [c]lose, [+w]aist, [-w]aist, [+s]oulder, [-s]houlder, [+e]lbow, [-e]lbow, [+r]ist, [-r]ist, [p]oints, [a]ngles\n ")
            if self.mode == "h":
                self.robot.arm.go_to_home_pose()
            elif self.mode == "s":
                self.robot.arm.go_to_sleep_pose()
            elif self.mode == "o":
                self.open()
            elif self.mode == "c":
                self.close()
            elif self.mode == "+w":
                self.move_joint('waist', 0.25)
            elif self.mode == "-w":
                self.move_joint('waist', -0.25)
            elif self.mode == "+s":
                self.move_joint('shoulder', 0.25)
            elif self.mode == "-s":
                self.move_joint('shoulder', -0.25)
            elif self.mode == "+e":
                self.move_joint('elbow', 0.25)
            elif self.mode == "-e":
                self.move_joint('elbow', -0.25)
            elif self.mode == "+r":
                self.move_joint('wrist_angle', 0.25)
            elif self.mode == "-r":
                self.move_joint('wrist_angle', -0.25)
            elif self.mode == "p":
                self.get_gripper_coords()
            elif self.mode == "a":
                self.get_joint_angles()
            elif self.mode == "f":
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


def main():
    with Robot() as rob:
        rob.run_robot_poses()


if __name__ == "__main__":
    main()
