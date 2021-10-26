#!/usr/bin/env python
#coding:utf-8
import numpy as np
import tf
import rospy
import math
from geometry_msgs.msg import  PoseStamped, Twist 
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from scipy.optimize import minimize
import time
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
# 线速度pid参数
Kp = 0.3
Ki = 0.01
Kd = 0.5
# 角速度pid参数
Kp1 = 1.7
Ki1 = 0.01
Kd1 = 0.1

dt = 0.1  # 时间间隔，单位：s
k = 1   # 速度系数

class PoseToPose_PID(object): # PoseToPose PID 类
    def __init__(self): # 初始化
        rospy.init_node('PoseToPose_PID', log_level=rospy.DEBUG) # 初始化节点 'PoseToPose_MPC'
        rospy.Subscriber("/odom", Odometry, self.callbackOdom, queue_size = 1) # 订阅 /rexrov/pose_gt 节点 （里程计）
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callbackGoal, queue_size=1) # 订阅 /move_base_simple/goal 节点 （目标点）
        # rospy.Subscriber("lidar/scan", LaserScan,callback, self..callbackScan, queue_size = 1)
	
        self.path_pub = rospy.Publisher('horizon_path', Path, queue_size=1) # 发布路径节点
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1) # 发布速度控制节点
        # self.obst_pub = rospy.Publisher("obstacle", Marker, queue_size = 1) # 发布障碍节点 
        self.odom = Odometry() # 里程计
       	self.goal = PoseStamped() # 目标点
        self.yaw_goal = 0 # 偏航角 目标
        self.dt = 0.2 # 时间间隔
	self.Rate = 30
        last_rotation = 0
        linear_speed = 1 # kp_distance
        angular_speed = 1 # kp_angular


        
        (goal_x, goal_y, goal_z) = self.getkey()
        if goal_z > 180 or goal_z < -180:
            print("you input wrong z range.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)
 
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        #distance is the error for length, x,y
        distance = goal_distance
        previous_distance = 0
        total_distance = 0

        previous_angle = 0
        total_angle = 0
       

        while distance > 0.05:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            #path_angle = error
            path_angle = atan2(goal_y - y_start, goal_x- x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation


            diff_angle = path_angle - previous_angle
            diff_distance = distance - previous_distance

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))

            control_signal_distance = kp_distance*distance + ki_distance*total_distance + kd_distance*diff_distance

            control_signal_angle = kp_angle*path_angle + ki_angle*total_angle + kd_distance*diff_angle

            move_cmd.angular.z = (control_signal_angle) - rotation
            #move_cmd.linear.x = min(linear_speed * distance, 0.1)
            move_cmd.linear.x = min(control_signal_distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            previous_distance = distance
            total_distance = total_distance + distance
            print("Current positin and rotation are: ", (position, rotation))

        (position, rotation) = self.get_odom()
        print("Current positin and rotation are: ", (position, rotation))

        print("reached :)   ^_^")

        while abs(rotation - goal_z) > 0.05:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            self.cmd_vel.publish(move_cmd)
            r.sleep()


        # rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        return

    def getkey(self):
        global x_input, y_input, z_input
        x = x_input
        y = y_input
        z = z_input
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

coord = [1,2]

lower_angle = 50

upper_angle = 80
angle = np.random.uniform(lower_angle, upper_angle, 1)   
print('Initial starting angle Theta wrt +X axis: ', angle[0])   

#initial_position = coord + angle
initial_position = np.concatenate((coord,angle))

#print('(X, Y, Theta):' ,coord[0], coord[1], angle[0])
print('Initial pose is:-')
print('(X, Y, Theta):', initial_position[0], initial_position[1], initial_position[2])

print("Enter final x position")
x_final = input()
print("Enter final y position")
y_final = input()
print("Enter final angle position")
angle_final = input()

final = [x_final, y_final, angle_final]
final_position = np.array(final)

x_input = final_position[0]
y_input = final_position[1]
z_input = final_position[2]


q = quaternion_from_euler(0, 0, initial_position[2])
# state_msg is an object
state_msg = ModelState()
state_msg.model_name = 'turtlebot3_waffle'
state_msg.pose.position.x = initial_position[0]
state_msg.pose.position.y = initial_position[1]
state_msg.pose.position.z = 0

state_msg.pose.orientation.x = q[0]
state_msg.pose.orientation.y = q[1]
state_msg.pose.orientation.z = q[2]
state_msg.pose.orientation.w = q[3]

set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
resp = set_state(state_msg)
print(resp)

time.sleep(5)

while not rospy.is_shutdown():
    PoseToPose_PID()

