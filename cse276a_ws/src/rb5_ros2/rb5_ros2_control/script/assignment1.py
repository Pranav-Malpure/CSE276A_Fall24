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
lx = 0.0675 #Horizontal distance between wheel axis and vertical axis of the car
ly = 0.057 # Vertical distance between the wheel axis and horizontal axis of the car
rw = 0.03 #Radius of the wheel

def get_under_range(theta): #Range is [-pi, pi)
    theta %= 6.28; 
    theta = (theta + 6.28) % 6.28;  
    if theta > 3.14:
        theta -= 6.28
    if theta == 3.14:
        theta = -3.14
    return theta

def calc_diff_theta(theta_target, theta):
    diff = theta_target - theta
    if diff >= 3.14:
        diff -= 6.28
    elif diff < -3.14:
        diff += 6.28
    return diff

def follow_waypoints(waypoints):
    global current_pose, Kv, sleep_time, Ktheta, threshold_distance
    controller = MegaPiController()
    waypoints_index = 0
    linear_distance = np.sqrt((waypoints[0] - current_pose[0])**2 + (waypoints[1] - current_pose[1])**2) #initializing the linear distance
    print("hello1")
    while True:
        print("hello2")

        if linear_distance < threshold_distance:
            waypoints_index = waypoints_index + 1
            if waypoints_index == len(waypoints):
                controller.carStop()
                print("hello3")
                break               
        print("hello4")
        current_waypoint = waypoints[waypoints_index]
        linear_distance = np.sqrt((current_waypoint[0] - current_pose[0])**2 + (current_waypoint[1] - current_pose[1])**2)
        print("hello4v")

        v_target = Kv * linear_distance
        theta_target = np.arctan2(current_waypoint[1] - current_pose[1], current_waypoint[0] - current_pose[0])
        gamma = Ktheta * calc_diff_theta(theta_target, current_pose[2])
        
        vx = v_target * np.cos(current_pose[2])
        vy = v_target * np.sin(current_pose[2])
        omegaz = gamma/sleep_time
        #controller.carStraight(v_target)
        #controller.carRotate(gamma/sleep_time)
        omega1 = (1 / rw) * (vx - vy - (lx+ly)*omegaz)
        omega2 = (1 / rw) * (vx + vy + (lx+ly)*omegaz)
        omega3 = (1 / rw) * (vx + vy - (lx+ly)*omegaz)
        omega4 = (1 / rw) * (vx - vy + (lx+ly)*omegaz)

        
        controller.setFourMotors(-omega1, omega2, omega3, -omega4)
        time.sleep(sleep_time)
        print("hello5")

        #current_pose[2] = (current_pose[2] + theta_target/sleep_time * sleep_time) #Find out theta range from instructors
        current_pose[2] = get_under_range(current_pose[2] + gamma)
        current_pose[0] = current_pose[0] + v_target * np.cos(current_pose[2]) * sleep_time
        current_pose[1] = current_pose[1] + v_target * np.sin(current_pose[2]) * sleep_time
    

def main():
    global waypoints, initial_time, initial_pose
    initial_time = time.time()
    initial_pose = [0,0]
    follow_waypoints(waypoints)

if __name__ == "__main__":
    main()