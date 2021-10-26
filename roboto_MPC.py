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
# 输入初始位姿、目标位姿、障碍物的位置; 输出是油门和方向盘。
class PoseToPose_MPC(object): # PoseToPose Mpc 类
    def __init__(self): # 初始化
        rospy.init_node('PoseToPose_MPC', log_level=rospy.DEBUG) # 初始化节点 'PoseToPose_MPC'
        rospy.Subscriber("/odom", Odometry, self.callbackOdom, queue_size = 1) # 订阅 /rexrov/pose_gt 节点 （里程计）
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callbackGoal, queue_size=1) # 订阅 /move_base_simple/goal 节点 （目标点）
        # rospy.Subscriber("lidar/scan", LaserScan,callback, self..callbackScan, queue_size = 1)
	
        self.path_pub = rospy.Publisher('horizon_path', Path, queue_size=1) # 发布路径节点
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1) # 发布速度控制节点
        self.obst_pub = rospy.Publisher("obstacle", Marker, queue_size = 1) # 发布障碍节点 
        self.odom = Odometry() # 里程计
       	self.goal = PoseStamped() # 目标点
        self.obstacle = Marker() # 干扰
        self.yaw_goal = 0 # 偏航角 目标
        self.WheelBase = 0.4 # 轴距
        self.MaxPedal = 6.0 # 最大油门
        self.MaxSpeed = 10.0 # 最大速度
        self.MaxSteer = 0.8 # 最大驱动
        self.horizon = 20# predict steps 预测步
        self.dt = 0.2 # 时间间隔
        self.Rate = 30 # 比率


    def callbackOdom(self, msg): # 回调 里程计
        self.odom = msg	
	#if msg.pose.pose.position.x - self.goal.pose.position.x <= 0.1 and msg.pose.pose.position.y - self.goal.pose.position.y <= 0.1:
	#	vel_cmd = 0
		
    def callbackGoal(self, msg): # 回调 目标
        self.goal = msg
        quaternion = [self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z, self.goal.pose.orientation.w]
	# 四元组 = x方向的目标姿态 y方向的目标姿态 z方向的目标姿态 w方向的目标姿态
        euler = tf.transformations.euler_from_quaternion(quaternion)
	# 欧拉 = 四元组的欧拉转换（四元组的值）
        self.yaw_goal = euler[2]
	# 目标角 = 欧拉[2]
    # def callbackScan(self, msg):


    def visualizeObstacle(self):
	# 可视化干扰
            self.obstacle.header.frame_id = "odom" 
            self.obstacle.id = 0
            self.obstacle.type = Marker.CYLINDER
            self.obstacle.scale.x = 0.5
            self.obstacle.scale.y = 0.5
            self.obstacle.scale.z = 1.0
            self.obstacle.action = Marker.ADD
            self.obstacle.pose.position.x = 5.0
            self.obstacle.pose.position.y = 5.0
            self.obstacle.pose.position.z = 0.0
            self.obstacle.pose.orientation.x = 0.0 # 干扰x方向的姿态
            self.obstacle.pose.orientation.y = 0.0 # 干扰y方向的姿态
            self.obstacle.pose.orientation.z = 0.0 # 干扰z方向的姿态
            self.obstacle.pose.orientation.w = 1.0 # 干扰w方向的姿态
            self.obstacle.color.r = 0.0 # 干扰的颜色 红色 0.0
            self.obstacle.color.g = 0.8 # 干扰的颜色 绿色 0.8
            self.obstacle.color.b = 0.0 # 干扰的颜色 蓝色 0.0
            self.obstacle.color.a = 0.5 # 干扰的颜色 透明度 0.5
            self.obstacle.lifetime = rospy.Duration() # 干扰 寿命 = rospy.持续时间（）
            self.obst_pub.publish(self.obstacle) # obst_pub发布(干扰)

	# 车辆模型
    def carModel(self, prev_state, dt, pedal, steering): # 上一个时态的状态，时间间隔,油门(加速度)，方向盘（转向）
        x = prev_state[0] # x = 状态列表 [0] x
        y = prev_state[1] # y = 状态列表 [1] y 	
        yaw = prev_state[2] # yaw = 状态列表 [2] yaw
        v = prev_state[3] # v=状态列表 [3] vx

        x += v * np.cos(yaw) * dt # x = x + v * cos(yaw)
        y += v * np.sin(yaw) * dt # y = y + v * sin(yaw)
        yaw += v * np.tan(steering)/self.WheelBase * dt # yaw = yaw + v * tan(steering) / WheelBase *dt
        v += pedal * dt - v/25.0 # v = v + pedal * dt  - v / 25.0

        return [x, y, yaw, v] # 返回 [x，y，yaw，v]

    def costFunction(self, u, *args): # 损失函数（u, *args） 目标函数
	# quanternion = [里程计的x方向目标姿态，里程计的y方向目标姿态，里程计的z方向目标姿态，里程计的w方向目标姿态]
        quanternion = [self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]
	# 欧拉转角 = queternion的欧拉转换（queternion的值）
        euler = tf.transformations.euler_from_quaternion(quanternion)
	# yaw = 欧拉[2]
        yaw = euler[2]
	# 状态 = [里程计的x位置的姿态，里程计的y位置的姿态，偏航角，里程计的X轴线速度]
        state = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, yaw, self.odom.twist.twist.linear.x]
        # 目标位置 = [目标x坐标，目标y坐标，目标偏航角]
        ref = [self.goal.pose.position.x, self.goal.pose.position.y, self.yaw_goal]
        cost = 0.0
	# 预测步
        for k in range(0, self.horizon):# 0 - 20 循环
            v_start = state[3] # 当前时态的速度
            yaw_start = state[2]
	#state = carModel(状态，时间间隔，u[k*2], u[k*2+1])
            state = self.carModel(state, self.dt, u[k*2], u[k*2+1]) # 下个状态 =  当前状态， 时间间隔， 踏板（加速度），转向
	  
            # Position Cost 位置损失
            cost += abs(ref[0] - state[0])**2
            cost += abs(ref[1] - state[1])**2
	    #cost += math.sqrt((ref[0] - state[0])**2 + (ref[1] - state[1])**2)

            # Angle Cost 角损失
            cost += abs(ref[2] - state[2])**2

	    # 车辆到达目标位姿控制量尽量小
            # Acceleration Cost 加速度损失
            cost += 10 * abs(state[3] - v_start)**2

            # Steering Input Cost  转向输入损失 
            cost +=  10 * u[k*2+1]**2 * self.dt #cost = cost + 10u[k*2+1]**2
            # 不想让方向变化太剧烈；不想让倒车倾向太强
            # Steering Changing Cost
            #cost += 5 * abs(state[2] - yaw_start)**2

            # Backward Cost
            #cost += 10 * self.dt * abs(state[3]) if  state[3] < 0  else 0
  	    # obstacle 是避开障碍物
            # Obstacle Cost 干扰损失  
	   # 距离 = (x位置的姿态- 干扰x位置的姿态)**2 +(y位置的姿态- 干扰y位置的姿态) 
           # distance = (state[0] - self.obstacle.pose.position.x)**2 + (state[1]- self.obstacle.pose.position.y)**2
           # distance = np.sqrt(distance) 
          # if (distance > 2.0): 
             #   cost += 10
            # else:
             #       cost +=  1 / distance * 50           
           	# cost = cost + 1/distace *50
        return cost 

    def Optimizer(self): # 优化
        num_inputs = 2 # 数字输入
        u = np.zeros(self.horizon * num_inputs) # u = 生成包含[预测步 * num_inputs]个元素的零矩阵 
        bounds = [] # 界限
        for i in range(self.horizon): # i 从 预测步中 循环
            bounds += [[-self.MaxPedal, self.MaxPedal]] # 正负最大加速度  【两行两列的矩阵】
            bounds += [[-self.MaxSteer, self.MaxSteer]] # 正负最大转向
	# quanternion = [里程计的x方向目标姿态，里程计的y方向目标姿态，里程计的z方向目标姿态，里程计的w方向目标姿态]
        quanternion = [self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]
	# 欧拉转角 = queternion的欧拉转换（queternion的值）	
        euler = tf.transformations.euler_from_quaternion(quanternion)
	# yaw = oula[2]
        yaw = euler[2]
	# 状态 = [里程计的x位置的姿态，里程计的y位置的姿态，偏航角，里程计的x位置的姿态，里程计的转动线性x]
        state = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, yaw, self.odom.twist.twist.linear.x]
        #  参考 = [目标x位置的姿态，目标x位置的姿态，目标偏航角]
        ref = [self.goal.pose.position.x, self.goal.pose.position.y, self.yaw_goal]
	# 删除第一步的 速度 和 角速度        
	u = np.delete(u, 0)
        u = np.delete(u, 0)
	# 列表尾 加上速度 和 角速度
        u = np.append(u, u[-2])
        u = np.append(u, u[-2])
	# u初始迭代值 =  优化算法， 损失函数，u是控制量（油门，方向盘）（state，ref）目标函数参数，算法的名字 ，变量的界限 ， 迭代停止的精度 10*（-5）
        u_solution = minimize(self.costFunction, u, (state, ref), method='SLSQP', bounds=bounds, tol=1e-5)
        u = u_solution.x  # 目标值 输出两个值，一个是加速度值 一个是转向角度值

        state = self.carModel(state, self.dt, u[0], u[1])
	# 预测状态 = 状态列表
        predicted_state = np.array([state])
	# 预测步内循环：
        for i in range(1, self.horizon):
	# predicted = 上一个车辆时态的 
            predicted = self.carModel(predicted_state[-1], self.dt, u[2*i], u[2*i+1])
            predicted_state = np.append(predicted_state, np.array([predicted]), axis=0)

        return u, predicted_state # 返回 ，预测的状态

    def calculateTwistCommandHorizonPath(self): # 计算控制车辆矩阵预测步路径
        u, predicted_state = self.Optimizer() 
        # calculate twist command # 计算控制车辆矩阵
        pedal = u[0]
        steering = u[1]
        twistCmd = Twist()
	# v = linear.x + pedal * 1/30 (Rate 比率）
        vel_cmd = self.odom.twist.twist.linear.x + pedal * 1.0/self.Rate  # v = 里程计的转动线性x + pedal * 1/30
        if vel_cmd < -self.MaxSpeed:
            twistCmd.linear.x = -self.MaxSpeed
        elif vel_cmd > self.MaxSpeed:
            twistCmd.linear.x = self.MaxSpeed
        else:
            twistCmd.linear.x = vel_cmd

        twistCmd.angular.z = steering

        self.cmd_pub.publish(twistCmd)
        
        # calculate horizon path #计算预测步路径
        path = Path() 
        timestamp = rospy.Time.now()
        path.header.stamp = timestamp
        path.header.frame_id = "odom" 

        for i in range(self.horizon):
            pose = PoseStamped()
            pose.header.stamp = timestamp
            pose.header.frame_id = "odom"
            pose.pose.position.x = predicted_state[i][0]
            pose.pose.position.y = predicted_state[i][1]
            path.poses.append(pose)  

        self.path_pub.publish(path)



    def run(self): # 运行程序
        rate = rospy.Rate(self.Rate)   # 30hz
        while not rospy.is_shutdown(): # 如果rospy没截止一直执行
            self.visualizeObstacle() # 可视化干扰
            if self.odom and self.goal:  # 如果 实现Odom 和 goal
                self.calculateTwistCommandHorizonPath() # 计算控制车辆矩阵预测步路径
            rate.sleep()

if __name__ == '__main__':
    p2p_mpc = PoseToPose_MPC() 

    try: #捕获异常
        p2p_mpc.run()
    except rospy.ROSInterruptException:
        pass
