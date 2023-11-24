#!/usr/bin/python3
import rospy
from prius_msgs.msg import Control
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math, os, control
import numpy as np
import matplotlib.pyplot as plt

class Controller:
    def __init__(self) -> None:
        rospy.init_node("local_control")
        self.waypoints = np.array([(12.9611651140224113, -12), (17.9611651140224113, -12), 
                                   (22.9611651140224113, -12), (24.9611651140224113,-12), (30.9611651140224113, -12), (35.9611651140224113, -17),
                                    (40.9611651140224113, -22),(45.9611651140224113,-27), (50.9611651140224113,-32), (9999,9999)]) 
        self.yaw_track = [np.arctan2(-0.000086001, 1)]
        for way in range(1, len(self.waypoints)):
            self.yaw_track.append(np.arctan2((self.waypoints[way][1]-self.waypoints[way-1][1]), (self.waypoints[way][0] - self.waypoints[way-1][0])))
        self.v = 0
        self.current_x = 2.9611651140224113
        self.current_y = -11.99913999007642
        self.yaw = 0
        self.point=0
        self.kd = 1
        self.controller = rospy.Publisher("/prius", Control, queue_size=5)
        self.location = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.callback)
        self.rate = rospy.Rate(10)
        self.error_prev = 0
        self.theta_prev = 0
        self.theta_dot = 0
        self.theta = 0
        self.v_prev = 0
        self.a = 0
    
    def callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        v_x = msg.twist.twist.linear.x
        v_y = msg.twist.twist.linear.y
        self.w_z = msg.twist.twist.angular.y
        self.v = math.sqrt(v_x**2 + v_y**2)
        self.v_dir = np.arctan2(v_y, v_x)

        self.v_error_dir = self.v*np.cos(self.v_dir - self.yaw_track[self.point])
        self.a = self.v - self.v_prev
        quaternions = msg.pose.pose.orientation
        self.v_prev = self.v
        _,_,self.yaw = euler_from_quaternion([quaternions.x, quaternions.y, quaternions.z, quaternions.w])

    def state_calc(self):
        if np.int64(self.waypoints[self.point][0]) == np.int64(self.current_x) and np.int64(self.waypoints[self.point][1] == np.int64(self.current_y)):
            self.waypoints = np.delete(self.waypoints, self.point, axis=0)
        if np.round(self.waypoints[self.point][0]) == np.round(self.current_x) and np.round(self.waypoints[self.point][1] == np.round(self.current_y)):
            self.waypoints = np.delete(self.waypoints, self.point, axis=0)

        diff_x = [self.current_x - way_x for way_x in self.waypoints[:,0]]
        diff_y = [self.current_y - way_y for way_y in self.waypoints[:,1]]
        diff_dist = np.hypot(diff_x, diff_y)
        self.point = np.argmin(diff_dist)
        self.error = np.hypot((self.current_x - self.waypoints[self.point][0]),(self.current_y - self.waypoints[self.point][1]))
        self.error_dot = self.error - self.error_prev   
        self.theta = self.yaw_track[self.point] - self.yaw
        self.theta_dot = self.theta - self.theta_prev
        state = np.array([[self.error],[self.error_dot],[self.theta],[self.theta_dot]])
        return state
    
    def a_and_b(self):
        if self.v != 0 or (1/np.tan(self.theta*0.5)) <20:
            A = np.array([[1/10, 1, 0, 0],[0, self.a - np.sin(self.theta*0.5)*0.5*self.theta_dot + self.theta_dot*self.v*(1/np.tan(self.theta*0.5))*0.5, -self.v*np.sin(self.theta*0.5), 0],[0, 0, 0, 1],[0, 0, 0, 0]])
            print("hi")
        else:
            A = np.array([[1/30, 1, 0, 0],[0, 1, 1-self.v*np.sin(self.theta*0.5), 0],[0, 0, 0, 1],[0, 0, 0, 0]])
        B = np.zeros((4, 1))
        B[3, 0] = self.v / 2.5    
        return A, B
    
    def k_calc(self):
        A,B = self.a_and_b()
        Q = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        R = np.array([[10000]])
        S,_,_ = control.care(A,B,Q,R)
        k = np.matmul(np.linalg.inv(R), B.transpose())
        k = np.matmul(k, S)
        return k

    def u_calc(self):
        k = self.k_calc()
        x = self.state_calc()
        return -np.arctan2(np.matmul(k, x), 1)
    
    def vel_control(self):
        max_throttle = 3
        vel_diff = max_throttle - self.v
        if self.v > max_throttle:
            throttle = 0.0
            brake = math.sqrt(vel_diff**2) * self.kd
        else:
            throttle = vel_diff*self.kd
            brake = 0.0
        print("throttle:", throttle, "break:", brake)
        return throttle, brake

    def main(self):
        while not rospy.is_shutdown():
            c = Control()
            if self.waypoints[self.point][0] == self.waypoints[-1][0] and self.waypoints[self.point][1] == self.waypoints[-1][1]:
                print("reached the destination")
                c.throttle = 0
                c.brake = 1
                self.controller.publish(c)
                break
            c.throttle, c.brake = self.vel_control()
            c.steer = self.u_calc()
            print("steer:", c.steer)
            self.controller.publish(c)
            # x_points=np.append(x_points,self.current_x)
            # y_points=np.append(y_points,self.current_y)
            self.rate.sleep() 


if __name__ == "__main__":
    t = Controller()
    t.main()
