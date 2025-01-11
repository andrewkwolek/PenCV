import multiprocessing
import time
import cv2
import pickle
import numpy as np
from camera import Camera
from arm import Robot


def camera_process(queue, resolution, fps, vals, stop_event):
    with Camera(resolution, fps, vals["min_hue"], vals["depth"], vals["min_sat"], vals["min_val"], vals["max_hue"], vals["max_sat"], vals["max_val"]) as cam:
        while not stop_event.is_set():
            P = cam.pipeline_iteration(display=True)
            key = cv2.waitKey(1)
            if P is not None:
                if not queue.empty():
                    queue.get()  # Remove old frame
                print(f"Put: {P}")
                queue.put(P)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break


def robot_process(queue, vals, stop_event):
    with Robot() as rob:
        R = vals["R"]
        t = vals["t"]

        rob.robot.arm.go_to_sleep_pose(moving_time=1)
        time.sleep(1)
        rob.robot.gripper.set_pressure(0.75)
        rob.robot.gripper.release()

        while not stop_event.is_set():
            if not queue.empty():
                P = queue.get()
                print(f"Get {P}")
                if P is not None:
                    Q = rob.find_translation(R, P, t)
                    # Q[1] += 0.1
                    print(f"Q_pen: {Q}")
                    Q_ee = rob.robot.arm.get_ee_pose()
                    print(Q_ee)
                    Q_ee = [Q_ee[0][3], Q_ee[1][3], Q_ee[2][3]]
                    error = np.subtract(Q, Q_ee)
                    print(f"Error: {error}")

                    e_mag = np.linalg.norm(error)
                    print(f"Error magnitude: {e_mag}")

                    # Move the arm to the target position
                    rob.robot.arm.set_ee_pose_components(
                        Q[0], Q[1], Q[2], moving_time=5)

                    # Check if the error magnitude is sufficiently small
                    if e_mag < 0.05:  # You can adjust this threshold as needed
                        rob.robot.gripper.grasp()  # Close the gripper
                        time.sleep(1)
                        rob.robot.arm.go_to_home_pose(moving_time=2)
                        time.sleep(0.25)
                        rob.robot.arm.go_to_sleep_pose()  # Return to sleep pose
                        break


def main():
    resolution = [640, 480]
    fps = 30
    vals = {}
    with open('cal.pkl', 'rb') as file:
        vals = pickle.load(file)
        print(vals)

    # Create a Queue for communication between processes
    queue = multiprocessing.Queue()

    # Create a stop event to signal when to stop the processes
    stop_event = multiprocessing.Event()

    # Start the camera and robot processes
    camera_proc = multiprocessing.Process(
        target=camera_process, args=(queue, resolution, fps, vals, stop_event))
    robot_proc = multiprocessing.Process(
        target=robot_process, args=(queue, vals, stop_event))

    camera_proc.start()
    robot_proc.start()

    try:
        # Wait for both processes to finish or until KeyboardInterrupt
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Stopping processes.")
    finally:
        # Trigger the stop event to signal both processes to stop
        stop_event.set()
        # Wait for both processes to finish
        camera_proc.join()
        robot_proc.join()
        print("Main process finished.")


if __name__ == "__main__":
    main()
