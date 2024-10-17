#!/usr/bin/env python3
""" MegaPi Controller ROS2 Wrapper"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from mpi_control import MegaPiController
import numpy as np
import time



# waypoints = [
# [0,0,0],
# [-1,0,0],
# [-1,1,1.57],
# [-2,1,0],
# [-2,2,-1.57],
# [-1,1,-0.78],
# [0,0,0]
# ]

waypoints = [
[0,0,0],
[-1/2,0,0],
[-1/2,1/2,1.57],
[-2/2,1/2,0],
[-2/2,2/2,-1.57],
[-1/2,1/2,-0.78],
[0,0,0]
]


initial_time = 0
current_pose = [0,0,0]
sleep_time = 0.5


Kv = 0.1 # this is the factor which gets multiplied with linear velocity to give the number to pass to the carStraight function, has to be callibrated
Ktheta = 0.16 # this is the factor which gets multiplied with angular velocity to give the number to pass to the carRotate function, has to be callibrated
threshold_distance = 0.1 # callibrated depending on how fine you want the car to follow the path
lx = 0.0675 #Horizontal distance between wheel axis and vertical axis of the car
ly = 0.057 # Vertical distance between the wheel axis and horizontal axis of the car
rw = 0.03 #Radius of the wheel
angular_vel_rotate = 80
linear_vel_straight = 80


class MegaPiControllerNode(Node):
    def __init__(self, verbose=False, debug=False):
        super().__init__('megapi_controller_node')
        self.mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=verbose)

    # This is a ROS2 Foxy template for MegaPi controller


    def get_under_range(self, theta): #Range is [-pi, pi)
        theta %= 6.28; 
        theta = (theta + 6.28) % 6.28;  
        if theta > 3.14:
            theta -= 6.28
        if theta == 3.14:
            theta = -3.14
        return theta

    def calc_diff_theta(self, theta_target, theta):
        diff = theta_target - theta
        if diff >= 3.14:
            diff -= 6.28
        elif diff < -3.14:
            diff += 6.28
        return diff

    def map_omegas(self, omega1, omega2, omega3, omega4):
        # return values between 30 to 60 in proportion of the current omega values

        # sum = abs(omega1) + abs(omega2) + abs(omega3) + abs(omega4)
        sum = 50
        ret_omega1 = int(int(omega1*8.35/sum)//2 + 30*np.sign(omega1))
        ret_omega2 = int(int(omega2*8.35/sum)//2 + 30*np.sign(omega2))
        ret_omega3 = int(int(omega3*8.35/sum)//2 + 30*np.sign(omega3))
        ret_omega4 = int(int(omega4*8.35/sum)//2 + 30*np.sign(omega4))
        # ret_omega1 = int(omega1*8.35)
        # ret_omega2 = int(omega2*8.35)
        # ret_omega3 = int(omega3*8.35)    
        # ret_omega4 = int(omega4*8.35)


        return [ret_omega1, ret_omega2, ret_omega3, ret_omega4]

    def follow_waypoints(self, waypoints):
        global current_pose, Kv, sleep_time, Ktheta, threshold_distance, angular_vel_rotate, linear_vel_straight
        print("initial hello from function follow waypoints")
        waypoints_index = 0
        linear_distance = np.sqrt((waypoints[0][0] - current_pose[0])**2 + (waypoints[0][1] - current_pose[1])**2) #initializing the linear distance
        print("hello1")
        try:
            while True:
                print ("linear_dst",linear_distance)
                print ("threshold_distance",threshold_distance)
                if linear_distance < threshold_distance:
                    # if self.calc_diff_theta(waypoints[waypoints_index][2], current_pose[2]) > 0.05:
                    #     # calculate omega that will make the robot rotate towards the waypoint
                    #     self.mpi_ctrl.setFourMotors(angular_vel_rotate)
                    #     time.sleep(0.5)
                    #     self.mpi_ctrl.carStop()
                    # current_pose[2] = waypoints[waypoints_index][2]

                    waypoints_index = waypoints_index + 1
                    time.sleep(1)
                    # if waypoints_index == len(waypoints):
                    #         self.mpi_ctrl.carStop()
                    #         break     
                    
                    # if self.calc_diff_theta(theta_target, current_pose[2]) > 0.05:
                    #     # calculate omega that will make the robot rotate towards the waypoint
                    #     self.mpi_ctrl.setFourMotors(angular_vel_rotate)
                    #     time.sleep(0.5)
                    #     self.mpi_ctrl.carStop()
                    # current_pose[2] = theta_target 
                    
                # linear_distance = np.sqrt((waypoints[waypoints_index][0] - current_pose[0])**2 + (waypoints[waypoints_index][1] - current_pose[1])**2)

                # add a while loop for the below, for loop maybe
                # self.mpi_ctrl.setFourMotors(-linear_vel_straight, linear_vel_straight, linear_vel_straight, -linear_vel_straight)
                # time.sleep(0.5) #callibrate this
                # self.mpi_ctrl.carStop()

                current_waypoint = waypoints[waypoints_index]
                print("current_waypoint = ", current_waypoint)
                print("current_pose = ", current_pose)            
                linear_distance = np.sqrt((current_waypoint[0] - current_pose[0])**2 + (current_waypoint[1] - current_pose[1])**2)
                print("linear_distance = ", linear_distance)

                theta_target = np.arctan2(current_waypoint[1] - current_pose[1], current_waypoint[0] - current_pose[0])


                v_target = Kv * linear_distance
                print("theta_target = ", theta_target)
                gamma = Ktheta * self.calc_diff_theta(theta_target, current_pose[2])
                print("gamma = ", gamma)
                
                vx = v_target * np.cos(current_pose[2])
                print("vx = ", vx)
                vy = v_target * np.sin(current_pose[2])
                print("vy = ", vy)
                
                omegaz = gamma/sleep_time
                #self.mpi_ctrl.carStraight(v_target)
                #self.mpi_ctrl.carRotate(gamma/sleep_time)
                omega1 = (1 / rw) * (vx - vy - (lx+ly)*omegaz)
                omega2 = (1 / rw) * (vx + vy + (lx+ly)*omegaz)
                omega3 = (1 / rw) * (vx + vy - (lx+ly)*omegaz)
                omega4 = (1 / rw) * (vx - vy + (lx+ly)*omegaz)
                motor_values = self.map_omegas(omega1, omega2, omega3, omega4)
                print("motor_values = ", motor_values)
                # TODO: Call self.mpi_ctrl's setFourMotors(self, vfl=0, vfr=0, vbl=0, vbr=0) method, but clarify why some of the parameters are being passed as negative to the motor
                #time.sleep(1)
                self.mpi_ctrl.setFourMotors(motor_values[0], motor_values[1], motor_values[2], motor_values[3])
                #print("this is also working>>>")

                # self.mpi_ctrl.setFourMotors(-100, 100, 100, -100)
                time.sleep(sleep_time)
                self.mpi_ctrl.carStop()
                #current_pose[2] = (current_pose[2] + theta_target/sleep_time * sleep_time) #Find out theta range from instructors
                current_pose[2] = self.get_under_range(current_pose[2] + gamma)
                current_pose[0] = current_pose[0] + v_target * np.cos(current_pose[2]) * sleep_time
                current_pose[1] = current_pose[1] + v_target * np.sin(current_pose[2]) * sleep_time
        
        except KeyboardInterrupt:
            print("Control-c pressed")
            self.mpi_ctrl.carStop()
            self.mpi_ctrl.close()




if __name__ == "__main__":
    # waypoints, initial_time, initial_pose
    initial_time = time.time()
    Kv = float(input("enter kv: "))
    Ktheta = float(input("enter ktheta: "))
    print('before init')
    rclpy.init()
    print("hello from main 1")
    mpi_ctrl_node = MegaPiControllerNode()
    print("hello from main 2")
    # Start ROS2 node
    try:
        mpi_ctrl_node.follow_waypoints(waypoints)
    except KeyboardInterrupt:
        print("Control-c pressed")
        mpi_ctrl_node.mpi_ctrl.close()
        rclpy.shutdown()
    
    rclpy.spin(mpi_ctrl_node) # Spin for until
    # Destroy node and shutdown when done. (Optional, as node would be cleaned up by garbage collection)
    mpi_ctrl_node.destroy_node()
    rclpy.shutdown()



# _________________________________________________________________________________________________________
'''BACKUP CODE:

    #!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import time
from geometry_msgs.msg import Point, Twist
from ackermann_msgs.msg import AckermannDriveStamped
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button


global current_pose, l1, l2
current_pose = [0,0,0]
global start_time
global previous_t
l1 = 1
l2 = 0


## robot pose 
def newOdom(msg):
    global x, y, theta, current_pose

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation

    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    current_pose = [x, y, theta]

def trajectory_function(x):
    return x**3 - 9*x*x + 22*x - 11

def trajectory_function_derivative(x):
    return 3*x**2 - 18*x + 22

def vector_angle(x, y):
    # Calculate the angle in radians using arctangent (atan2)
    angle_radians = math.atan2(y, x)    
    
    return angle_radians    
## Main Node
def controller():
    rospy.init_node('main_controller', anonymous=True)
    global pose, start_time
    x_values = np.linspace(1, 5.5, 1000)
    y_values = trajectory_function(x_values)
    slopes = trajectory_function_derivative(x_values)
    Path = np.array(list(zip(x_values, y_values, slopes)))

    for t in range(len(Path)):
        gamma = float(input("Input Gamma: ")) # ideally = 0.2
        h = float(input("Input h: ")) # ideally = 1.6
        k = float(input("Input k: ")) # ideally = 2

        x = Path[t][0]
        y = Path[t][1]
        phi = Path[t][2]

        start_time = rospy.Time.now()

        plot_x_position = []
        plot_y_position = []
        plot_phi = []
    
        previous_t  = 0
        sub = rospy.Subscriber("/odom", Odometry, newOdom)
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        speed = Twist()
        r = rospy.Rate(4)
        goal = Point()

        start_time = rospy.Time.now()

        plot_x_position = []
        plot_y_position = []
        plot_phi = []
    
        previous_t  = 0

        while(True):

            current_time = rospy.Time.now()
            t = (current_time).to_sec() - (start_time).to_sec()
            print("pose", current_pose)
            x_p = current_pose[0]
            y_p = current_pose[1]
            phi_p = current_pose[2]

            plot_x_position.append(x_p)
            plot_y_position.append(y_p)
            plot_phi.append(phi_p)

            dx = x-x_p
            dy = y-y_p

            e = np.sqrt(dx**2 + dy**2)
            theta = vector_angle(dx, dy)
            alpha = theta - phi_p
            theta = theta - phi
            

            u = gamma*np.cos(alpha)*e
            omega = k*alpha + gamma*np.cos(alpha)*(np.sin(alpha))*(alpha + h*theta)/alpha

            speed.linear.x = u
            speed.angular.z = omega

            print("-------------------------------------")
            print("time ", t)
            print("velocity ", speed.linear.x)
            print("steering_rate ", speed.angular.z)
            print("-------------------------------------")
            print('\n')
            pub.publish(speed)
            previous_t = t
            r.sleep()
            
            lambda_ = 0.002
            epsilon  = np.pi*np.pi/8
            print("current error ", lambda_*e*e + (alpha**2 + h*theta*theta))
            if (lambda_*e*e + (alpha**2 + h*theta*theta)) < epsilon:
                print("NEXT STEP_________________________________")
                break
            
        

        

    # plt.axis('equal')
    # plt.plot(plot_x_position, plot_y_position, label='Robot Trajectory')
    # plt.plot()
    
    # plt.grid(True)
    # plt.show()


           

if __name__ == '__main__':
    try:
        time.sleep(1)
        controller()
    except rospy.ROSInterruptException:
        pass

'''