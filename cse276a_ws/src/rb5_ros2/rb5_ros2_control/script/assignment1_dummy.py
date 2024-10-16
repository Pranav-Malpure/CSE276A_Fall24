from rb5_ros2_control.mpi_control import MegaPiController
import numpy as np
import time
from rclpy.clock import Clock

waypoints = [
[-1,0,0],
[-1,1,1.57],
[-2,1,0],
[-2,2,-1.57],
[-1,1,-0.78],
[0,0,0]
]

initial_time = 0
current_pose = np.array([0,0,0])
sleep_time = 1

Kv = 1.5 # this is the factor which gets multiplied with linear velocity to give the number to pass to the carStraight function, has to be callibrated
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
    linear_distance = np.sqrt((waypoints[0][0] - current_pose[0])**2 + (waypoints[0][1] - current_pose[1])**2) #initializing the linear distance
    print("hello1")

    for t in range(len(waypoints)):
        x = waypoints[t][0]
        y = waypoints[t][1]
        theta_target = waypoints[t][2]

        start_time = time.time()


        while True:
            current_time = time.time()
            t = current_time - initial_time # have to check whether this is in s or ms
            
            print("current pose", current_pose)

            x_p = current_pose[0]
            y_p = current_pose[1]
            phi_p = current_pose[2]

            
            dx = x-x_p
            dy = y-y_p

            
            if linear_distance < threshold_distance:
                waypoints_index = waypoints_index + 1
                if waypoints_index == len(waypoints):
                    controller.carStop()
                    break               
            

            linear_distance = np.sqrt((current_waypoint[0] - current_pose[0])**2 + (current_waypoint[1] - current_pose[1])**2)

            t = (current_time).to_sec() - (start_time).to_sec()

            x = current_waypoint[0]
            y = current_waypoint[1]
            theta_target = current_waypoint[2]
            

            


            dx = x-x_p
            dy = y-y_p

            e = np.sqrt(dx**2 + dy**2)
            theta = vector_angle(dx, dy)
            alpha = theta - phi_p
            theta = theta - phi



            v_target = gamma*np.cos(alpha)*e
            omega = k*alpha + gamma*np.cos(alpha)*(np.sin(alpha))*(alpha + h*theta)/alpha


            lambda_ = 0.002
            epsilon  = np.pi*np.pi/8

            if (lambda_*e*e + (alpha**2 + h*theta*theta)) < epsilon:
                print("NEXT STEP_________________________________")
                break

            omega1 = (1 / rw) * (vx - vy - (lx+ly)*omegaz)
            omega2 = (1 / rw) * (vx + vy + (lx+ly)*omegaz)
            omega3 = (1 / rw) * (vx + vy - (lx+ly)*omegaz)
            omega4 = (1 / rw) * (vx - vy + (lx+ly)*omegaz)

            
            controller.setFourMotors(-omega1, omega2, omega3, -omega4)
            # time.sleep(sleep_time)
            print("hello5")

            #current_pose[2] = (current_pose[2] + theta_target/sleep_time * sleep_time) #Find out theta range from instructors
            current_pose[2] = get_under_range(current_pose[2] + gamma)
            current_pose[0] = current_pose[0] + v_target * np.cos(current_pose[2]) * sleep_time
            current_pose[1] = current_pose[1] + v_target * np.sin(current_pose[2]) * sleep_time
    

def main():
    global waypoints, initial_time
    
    follow_waypoints(waypoints)

if __name__ == "__main__":
    main()