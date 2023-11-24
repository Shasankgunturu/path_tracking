#!/usr/bin/python3

import rospy
from prius_msgs.msg import Control
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math, os
import numpy as np
import matplotlib.pyplot as plt

class Stan:
    def __init__(self):
        rospy.init_node("stanley")
        # self.waypoints = np.array([(12.9611651140224113, -12), (17.9611651140224113, -12), 
        #                            (22.9611651140224113, -12), (24.9611651140224113,-12), (30.9611651140224113, -12), (35.9611651140224113, -17),
        #                            (40.9611651140224113, -22),(45.9611651140224113,-27), (50.9611651140224113,-32), (9999,9999)]) 
        expanded_path = os.path.expanduser('~/test_ws/src/car_demo/car_demo/Path_Tracking_Algorithms_Sample/waypoints.txt')
        self.waypoints = []
        with open(expanded_path, 'r') as file:
            lines = file.readlines()
            for line in lines:
                parts = line.strip().split(",")
                x = float(parts[0])
                y = float(parts[1])
                self.waypoints.append((x,y))
                # self.waypoints = np.append(self.waypoints,np.array(x,y))
        self.waypoints.append((9999,9999))
        self.waypoints = np.array(self.waypoints)
        self.current_x = 2.9611651140224113
        self.current_y = -11.99913999007642
        self.yaw = 0
        self.point = 0
        self.yaw_points = [np.arctan2(-0.000086001, 1)]
        for way in range(1, len(self.waypoints)):
            self.yaw_points.append(np.arctan2((self.waypoints[way][1]-self.waypoints[way-1][1]), (self.waypoints[way][0] - self.waypoints[way-1][0])))
        self.yaw_points = np.array(self.yaw_points)
        print(self.yaw_points)
        self.controller = rospy.Publisher("/prius", Control, queue_size=5)
        self.location = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.callback)
        self.rate = rospy.Rate(30)
        self.k = 0.0125
        self.kd = 2
        self.v = 0
        self.max_throttle = 3

    def callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        v_x = msg.twist.twist.linear.x
        v_y = msg.twist.twist.linear.y
        self.v = math.sqrt(v_x**2 + v_y**2)
        quaternions = msg.pose.pose.orientation
        _,_,self.yaw = euler_from_quaternion([quaternions.x, quaternions.y, quaternions.z, quaternions.w])

    def vel_control(self):
        
        vel_diff = self.max_throttle - self.v
        # if self.v > self.max_throttle:
        #     throttle = 0.0
        #     brake = math.sqrt(vel_diff**2) * self.kd
        # else:
        throttle = vel_diff*self.kd
        brake = 0.0
        return throttle, brake
    
    def ind_calc(self):
        if math.sqrt((np.int64(self.waypoints[self.point][0]) - np.int64(self.current_x))**2) < 2 and math.sqrt((np.int64(self.waypoints[self.point][1]) - np.int64(self.current_y))**2) < 2:
            self.waypoints = np.delete(self.waypoints, self.point, axis=0)
            self.yaw_points = np.delete(self.yaw_points, self.point, axis=0)
        if math.sqrt((np.round(self.waypoints[self.point][0]) - np.round(self.current_x))**2) < 2 and math.sqrt((np.round(self.waypoints[self.point][1]) - np.round(self.current_y))**2) < 2:
            self.waypoints = np.delete(self.waypoints, self.point, axis=0)
            self.yaw_points = np.delete(self.yaw_points, self.point, axis=0)
        diff_x = [self.current_x - way_x for way_x in self.waypoints[:,0]]
        diff_y = [self.current_y - way_y for way_y in self.waypoints[:,1]]
        diff_dist = np.hypot(diff_x, diff_y)
        self.point = np.argmin(diff_dist)

        print("waypoints:", self.waypoints[self.point][0], self.waypoints[self.point][1])
        print("position:", self.current_x, self.current_y)

        
        print("point:",self.point)
        return self.point

    def heading_cal(self, ind):
        return 2*(self.yaw_points[ind] - self.yaw)
    
    def cross_track_calc(self, ind):
        # front_vector = [-np.cos(self.yaw+(np.pi/2)), -np.sin(self.yaw+(np.pi/2))]
        # curr_vec = [self.waypoints[ind][0], self.waypoints[ind][1]]
        # et = np.dot(curr_vec, front_vector)
        et = math.sqrt((self.current_y-self.waypoints[ind][1])**2 + (self.current_x-self.waypoints[ind][0])**2)
        if self.v==0:
            cross_error=0
        else:
            cross_error = np.arctan2((self.k*et),(self.v))
        print("cross:",cross_error)
        return cross_error
    
    def main(self):
        x_points=np.array([])
        y_points=np.array([])
        plot_waypoints = self.waypoints[:-1]
        while not rospy.is_shutdown():
            if self.waypoints[self.point][0] == self.waypoints[-1][0] and self.waypoints[self.point][1] == self.waypoints[-1][1]:
                print("reached the destination")
                c.throttle = 0
                c.brake = 1
                self.controller.publish(c)
                break
            ind = self.ind_calc()
            c = Control()
            c.steer = self.heading_cal(ind) + self.cross_track_calc(ind)
            c.throttle, c.brake = self.vel_control()
            self.controller.publish(c)
            x_points=np.append(x_points,self.current_x)
            y_points=np.append(y_points,self.current_y)
            if round(self.current_x) == self.waypoints[-1][0] and round(self.current_y) == self.waypoints[-1][1]:
                print("reached the destination")
                break
            print("steering_angle:",c.steer)
            self.rate.sleep()
        print("x_current_plot:",x_points)
        print("x_plot:",plot_waypoints[:,0])
        plt.plot(plot_waypoints[:,0], plot_waypoints[:,1], label="Original Path")
        plt.plot(x_points, y_points, linestyle="--", label="Pure Pursuit Algorithm")
        plt.xlabel("x_coordinate")
        plt.ylabel("y_coordinate")
        plt.show()



if __name__ == "__main__":
    t = Stan()
    t.main()