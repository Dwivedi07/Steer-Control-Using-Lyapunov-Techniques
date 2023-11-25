#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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
        self.yaw_angle = 0.0
        self.x_positions_list = []
        self.y_positions_list = []
        self.task_completed = False

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
        x_desired = 0
        y_desired = 0
        yaw_desired = 0

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

            if (error_distance < 0.01) and (abs(yaw_transition) < 0.05) and (not self.task_completed):
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.task_completed = True
                print("Task Completed")

                plt.plot(self.x_positions_list, self.y_positions_list, linestyle='-', color='blue', label='Robot Path')
                plt.plot([-1, 1], color='green', marker='x', label='Start Positions')
                plt.plot([x_desired, y_desired], color='red', marker='x', label='Goal Positions')
                plt.title("Parking")
                plt.xlabel("x")
                plt.ylabel("y")
                plt.legend('Path Traced')  
                plt.grid(True)  
                plt.show()

            elif not self.task_completed:
                self.speed.linear.x = linear_velocity
                self.speed.angular.z = angular_velocity


if __name__ == '__main__':
    controller = SpeedController()
    rate = rospy.Rate(4)
    while not rospy.is_shutdown():
    
        controller.calculate_speed()
        controller.publisher.publish(controller.speed)
        rate.sleep()
    
