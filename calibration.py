from camera import Camera
from arm import Robot
import cv2
import time


class Calibration:
    def __init__(self):
        self.hsv = {}
        self.img_coords = []
        self.robot_coords = []
        self.resolution = [640, 480]
        self.fps = 30

    def set_hsv(self):
        with Camera() as cam:
            while True:
                cam.pipeline_iteration()
                key = cv2.waitKey(1)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('c'):
                    self.hsv = cam.get_calibration_values()
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break

    def calibration_routine(self):
        with Robot() as rob:
            with Camera(resolution=self.resolution, fps=self.fps, min_hue=self.hsv["min_hue"], min_sat=self.hsv["min_sat"], min_val=self.hsv["min_val"], max_hue=self.hsv["max_hue"], max_sat=self.hsv["max_sat"], max_val=self.hsv["max_val"]) as cam:
                rob.robot.arm.go_to_home_pose(moving_time=1)
                time.sleep(1)
                rob.robot.gripper.set_pressure(0.75)
                rob.robot.gripper.grasp()
                time.sleep(1)

                self.set_hsv()

                # Get coordinates from home pose
                cam_coord = cam.pipeline_iteration()
                while cam_coord == None:
                    cam_coord = cam.pipeline_iteration()
                self.img_coords.append(cam_coord)
                self.robot_coords.append(rob.get_gripper_coords())
                time.sleep(1)

                # Get coords from first pose
                rob.robot.arm.set_joint_positions([0.25, -0.25, -0.1, 0.25])
                time.sleep(1)
                cam_coord = cam.pipeline_iteration()
                while cam_coord == None:
                    cam_coord = cam.pipeline_iteration()
                self.img_coords.append(cam_coord)
                self.robot_coords.append(rob.get_gripper_coords())

                # Get coords from second pose
                rob.robot.arm.set_joint_positions([0.5, 0, -0.5, 0.5])
                time.sleep(1)
                cam_coord = cam.pipeline_iteration()
                while cam_coord == None:
                    cam_coord = cam.pipeline_iteration()
                self.img_coords.append(cam_coord)
                self.robot_coords.append(rob.get_gripper_coords())

                # Get coords from third pose
                rob.robot.arm.set_joint_positions([0.75, -0.2, -0.15, 0.3])
                time.sleep(1)
                cam_coord = cam.pipeline_iteration()
                while cam_coord == None:
                    cam_coord = cam.pipeline_iteration()
                self.img_coords.append(cam_coord)
                self.robot_coords.append(rob.get_gripper_coords())

                # Get coords from fourth pose
                rob.robot.arm.set_joint_positions([0.6, -0.5, -0.25, -0.25])
                time.sleep(1)
                cam_coord = cam.pipeline_iteration()
                while cam_coord == None:
                    cam_coord = cam.pipeline_iteration()
                self.img_coords.append(cam_coord)
                self.robot_coords.append(rob.get_gripper_coords())

                # Get coords from fifth pose
                rob.robot.arm.set_joint_positions([0.15, 0.2, -0.35, 1])
                time.sleep(1)
                cam_coord = cam.pipeline_iteration()
                while cam_coord == None:
                    cam_coord = cam.pipeline_iteration()
                self.img_coords.append(cam_coord)
                self.robot_coords.append(rob.get_gripper_coords())

                rob.robot.arm.go_to_home_pose()
                time.sleep(1)
                rob.robot.gripper.release()
                print(self.img_coords)
                print(self.robot_coords)

    def get_hsv(self):
        return self.hsv

# main loop for calibration


def main():
    calibrate = Calibration()
    calibrate.set_hsv()
    calibrate.calibration_routine()


if __name__ == "__main__":
    main()
