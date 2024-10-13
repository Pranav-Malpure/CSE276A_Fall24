from rb5_ros2_control.mpi_control import MegaPiController
import numpy as np
import time

waypoints = [
[0,0,0],
[-1,0,0],
[-1,1,1.57],
[-2,1,0],
[-2,2,-1.57],
[-1,1,-0.78],
[0,0,0]
]

initial_pose = [0,0]
initial_time = 0
current_pose = np.array([0,0,0])
sleep_time = 1

Kv = 0.5 # this is the factor which gets multiplied with linear velocity to give the number to pass to the carStraight function, has to be callibrated
Ktheta = 1 # this is the factor which gets multiplied with angular velocity to give the number to pass to the carRotate function, has to be callibrated
threshold_distance = 0.1 # callibrated depending on how fine you want the car to follow the path

def follow_waypoints(waypoints):
    global current_pose, Kv, sleep_time, Ktheta, threshold_distance
    controller = MegaPiController()
    waypoints_index = 0
    linear_distance = np.sqrt((waypoints[0] - current_pose[0])**2 + (waypoints[1] - current_pose[1])**2) #initializing the linear distance
    while True:
        if linear_distance < threshold_distance:
            waypoints_index = waypoints_index + 1
        current_waypoint = waypoints[waypoints_index]
        linear_distance = np.sqrt((current_waypoint[0] - current_pose[0])**2 + (current_waypoint[1] - current_pose[1])**2)
        v_target = Kv * linear_distance
        theta_target = np.arctan2(current_waypoint[1] - current_pose[1], current_waypoint[0] - current_pose[0])
        
        
        controller.carStraight(v_target)
        controller.carRotate(Ktheta*theta_target/sleep_time)

        
        time.sleep(sleep_time)
        current_pose[2] = (current_pose[2] + theta_target/sleep_time * sleep_time) #Find out theta range from instructors
        current_pose[0] = current_pose[0] + v_target * np.cos(current_pose[2]) * sleep_time
        current_pose[1] = current_pose[1] + v_target * np.sin(current_pose[2]) * sleep_time
        if waypoints_index == len(waypoints) - 1 and linear_distance < threshold_distance:
            controller.carStop()
            break
    

def main():
    global waypoints, initial_time, initial_pose
    initial_time = time.time()
    initial_pose = [0,0]
    follow_waypoints(waypoints)

if __name__ == "__main__":
    main()