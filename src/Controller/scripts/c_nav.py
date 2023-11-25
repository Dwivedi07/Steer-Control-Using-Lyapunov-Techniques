#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

global nav_points
nav_points  = [(-1,2),(-2,2),(-3,3),(-2,3),(-2,4),(-1,5),(-1,4),(-1,3),(0,3),(1,3),(0,4),(1,4)]

class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

class EulerAngles:
    def __init__(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

def quaternion_to_euler(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_angle = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_angle = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_angle = math.atan2(t3, t4)
    
    euler_angles = EulerAngles(roll_angle, pitch_angle, yaw_angle)
    return [euler_angles.roll, euler_angles.pitch, euler_angles.yaw]

class SpeedController:
    def __init__(self):
        rospy.init_node("speed_controller")
        self.subscriber = rospy.Subscriber("/odom", Odometry, self.handle_odometry)
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.speed = Twist()
        self.x_position = 0.0
        self.y_position = 0.0
        self.x_previous = -1
        self.y_previous = -1
        self.ci = 0
        self.pl = 0
        self.yaw_angle = 0.0
        self.x_positions_list = []
        self.y_positions_list = []
        self.task_completed = False
        
    def control_loop(self):
         rate = rospy.Rate(4)
         while not rospy.is_shutdown():
            controller.calculate_speed()
            print('Publishing',controller.speed.linear.x, controller.speed.angular.z)
            controller.publisher.publish(controller.speed)
            rate.sleep()
        
            
        
    def desired_loc(self):
        if (self.ci < len(nav_points)):
            # print("current index: ", self.ci)
            x_des = nav_points[self.ci][0]
            y_des = nav_points[self.ci][1]
        if (self.ci != len(nav_points) - 1):
            x_fut = nav_points[self.ci+1][0] 
            y_fut = nav_points[self.ci+1][1] 
        else:
            x_fut = nav_points[self.ci-1][0]
            y_fut = nav_points[self.ci-1][1]
        dist = ((x_fut - x_des)**2 + (y_fut - y_des)**2)**0.5
        if (y_fut- y_des) >= 0:
            phi_des = math.acos((x_fut - x_des)/dist)
        elif (y_fut - y_des) < 0 :
            phi_des = - math.acos((x_fut - x_des)/dist)

        return x_des, y_des, phi_des

    def angle_difference(self, angle1, angle2):
        fac = (math.sin(angle1 - angle2))
        if fac > 0:
            sign = 1
        elif fac < 0:
            sign = -1
        else:
            sign = 0
        return sign * math.acos(math.cos(angle1 - angle2))

    def handle_odometry(self, msg):
        self.x_position = msg.pose.pose.position.x
        self.y_position = msg.pose.pose.position.y
        quaternion = Quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler_angles = quaternion_to_euler(quaternion)
        self.yaw_angle = euler_angles[2]
        self.x_positions_list.append(self.x_position)
        self.y_positions_list.append(self.y_position)
        
        
    def calculate_speed(self):
        global nav_points
        if self.ci < len(nav_points):
            x_desired, y_desired, yaw_desired = self.desired_loc()

            gamma = 0.3
            k = 0.5
            h = 0.5

            error_distance = ((self.x_position - x_desired) ** 2 + (self.y_position - y_desired) ** 2) ** 0.5

            yaw_transition = self.angle_difference(self.yaw_angle, yaw_desired)

            if error_distance != 0:
                if (y_desired - self.y_position) >= 0:
                    theta = self.angle_difference(math.acos((x_desired - self.x_position) / error_distance), yaw_desired)
                elif (y_desired - self.y_position) < 0:
                    theta = self.angle_difference(-math.acos((x_desired - self.x_position) / error_distance), yaw_desired)

                alpha = self.angle_difference(theta, yaw_transition)

                linear_velocity = gamma * math.cos(alpha) * error_distance

                angular_velocity = (k * alpha) + (gamma * math.cos(alpha) * math.sin(alpha) * (alpha + h * theta)) / alpha
                print('Calculated', linear_velocity, angular_velocity,error_distance)
                self.speed.linear.x = linear_velocity
                self.speed.angular.z = angular_velocity
                if (error_distance < 0.01):
                    self.x_previous = nav_points[self.ci][0]
                    self.y_previous = nav_points[self.ci][1]
                    self.ci += 1

        else:
            
            self.speed.linear.x = 0
            self.speed.angular.z = 0
            self.task_completed = True
            if self.pl ==0:
                plt.plot(self.x_positions_list, self.y_positions_list, linestyle='-', color='blue')
                plt.scatter([j[0] for j in nav_points], [j[1] for j in nav_points], color='r')
                plt.title("Navigation Plot")
                plt.xlabel("x")
                plt.ylabel("y")
                plt.legend(['Path Traced'])
                plt.grid(True)
                plt.show()
                self.pl = 1

if __name__ == '__main__':
    controller = SpeedController()
    
    rate = rospy.Rate(4)
    while not rospy.is_shutdown():
        controller.calculate_speed()
        print('Publishing',controller.speed.linear.x, controller.speed.linear.y)
        controller.publisher.publish(controller.speed)
        rate.sleep()
