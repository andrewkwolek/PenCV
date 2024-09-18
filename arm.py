from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
# The robot object is what you use to control the robot

class Robot:
    def __init__(self):
        self.robot = InterbotixManipulatorXS("px100", "arm", "gripper")

    def __enter__(self):
        robot_startup()
        self.mode = 'h'
        return self
    
    def __exit__(self):
        self.robot.arm.go_to_sleep_pose()
        robot_shutdown()

    def get_mode(self):
        return self.mode
    
    def run(self):
        while self.mode != 'q':
            self.mode = input("[h]ome, [s]leep, [q]uit")
            if self.mode == 'h':
                self.robot.arm.go_to_home_pose()
            elif self.mode == 's':
                self.robot.arm.go_to_sleep_pose()
    
    
