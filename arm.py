from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup

import modern_robotics as mr
# The robot object is what you use to control the robot

class Robot:
    def __init__(self):
        self.robot = InterbotixManipulatorXS("px100", "arm", "gripper")

    def __enter__(self):
        robot_startup()
        self.mode = 'h'
        self.robot.arm.go_to_home_pose()
        return self
    
    def __exit__(self,  exc_type, exc_value, exc_traceback):
        self.robot.arm.go_to_sleep_pose()
        robot_shutdown()

    def get_mode(self):
        return self.mode
    
    def calibration_test(self, cam_coord):
        while self.mode != 'q':
            self.mode = input("[h]ome, [s]leep, [q]uit, [c]lose, [o]pen, [r]otate, [s]houlder, [e]lbow, [k]eep, [p]ass\n")
            if self.mode == 'h':
                self.robot.arm.go_to_home_pose()
            elif self.mode == 's':
                self.robot.arm.go_to_sleep_pose()
            elif self.mode == 'c':
                self.robot.gripper.grasp()
            elif self.mode == 'o':
                self.robot.gripper.release()
            elif self.mode == 'r':
                cur_pos = self.robot.arm.get_single_joint_command("waist")
                print(self.robot.arm.set_single_joint_position('waist', cur_pos + 0.25, moving_time=3))
                #self.robot.arm.set_single_joint_position('waist', 150, moving_time=10)
            elif self.mode == 's':
                self.robot.arm.set_single_joint_position('shoulder', 0.25, moving_time=3)
                print(f"Joint positions: {self.robot.arm.get_joint_commands()}")
            elif self.mode == 'e':
                self.robot.arm.set_single_joint_position('elbow', -0.25, moving_time=3)
                print(f"Joint positions: {self.robot.arm.get_joint_commands()}")
            elif self.mode == 'k':
                T = mr.FKinSpace(self.robot.arm.robot_des.M, self.robot.arm.robot_des.Slist, self.robot.arm.get_joint_commands())
                _, d = mr.TransToRp(T)
                print([d, cam_coord])
            elif self.mode == 'p':
                pass

    
    def run(self):
        while self.mode != 'q':
            self.mode = input("[h]ome, [s]leep, [q]uit, [d]ata, [c]lose, [o]pen, [r]otate, [s]houlder, [e]lbow\n")
            if self.mode == 'h':
                self.robot.arm.go_to_home_pose()
            elif self.mode == 's':
                self.robot.arm.go_to_sleep_pose()
            elif self.mode == 'd':
                print(f"Joint positions: {self.robot.arm.get_joint_commands()}")
                print(f"Commanded EE pose: {self.robot.arm.get_ee_pose_command()}")
                print(f"Waist pos: {self.robot.arm.get_single_joint_command("waist")}")
                print(f"Shoulder pos: {self.robot.arm.get_single_joint_command("shoulder")}")
                print(f"Elbow pos: {self.robot.arm.get_single_joint_command("elbow")}")
                print(f"Wrist angle pos: {self.robot.arm.get_single_joint_command("wrist_angle")}")
                T = mr.FKinSpace(self.robot.arm.robot_des.M, self.robot.arm.robot_des.Slist, self.robot.arm.get_joint_commands())
                rotation_mat = mr.TransToRp(T)
                # print(f"Rotation atrix and displacement: {rotation_mat}")
            elif self.mode == 'c':
                self.robot.gripper.grasp()
            elif self.mode == 'o':
                self.robot.gripper.release()
            elif self.mode == 'r':
                cur_pos = self.robot.arm.get_single_joint_command("waist")
                print(self.robot.arm.set_single_joint_position('waist', cur_pos + 0.25, moving_time=3))
                #self.robot.arm.set_single_joint_position('waist', 150, moving_time=10)
            elif self.mode == 's':
                self.robot.arm.set_single_joint_position('shoulder', 0.25, moving_time=3)
                print(f"Joint positions: {self.robot.arm.get_joint_commands()}")
            elif self.mode == 'e':
                self.robot.arm.set_single_joint_position('elbow', -0.25, moving_time=3)
                print(f"Joint positions: {self.robot.arm.get_joint_commands()}")
            


    
    
def main():
    with Robot() as arm:
        arm.run()

if __name__=="__main__":
    main()