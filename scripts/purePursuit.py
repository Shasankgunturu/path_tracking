#!/usr/bin/python3
import rospy
from prius_msgs.msg import Control
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math, os
import numpy as np
import matplotlib.pyplot as plt


class Pursuit:
    def __init__(self):
        rospy.init_node("pure_pursuit")
        # self.waypoints = np.array([(12.9611651140224113, -12), (17.9611651140224113, -12), 
        #                           (22.9611651140224113, -12), (24.9611651140224113,-12), (30.9611651140224113, -22),
        #                           (30.9611651140224113, -32),(30.9611651140224113,-42),(30.9611651140224113,-55), (9999,9999)])
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
        self.L = 3
        self.kd = 2
        self.v = 0
        self.current_x = 2.9611651140224113
        self.current_y = -11.99913999007642
        self.yaw = 0
        self.controller = rospy.Publisher("/prius", Control, queue_size=5)
        self.location = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.callback)
        self.rate = rospy.Rate(30)
        self.point = 0

    def callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        v_x = msg.twist.twist.linear.x
        v_y = msg.twist.twist.linear.y
        self.v = math.sqrt(v_x**2 + v_y**2)
        quaternions = msg.pose.pose.orientation
        _,_,self.yaw = euler_from_quaternion([quaternions.x, quaternions.y, quaternions.z, quaternions.w])

    
    def alpha(self):

        if np.int64(self.waypoints[self.point][0]) == np.int64(self.current_x) and np.int64(self.waypoints[self.point][1] == np.int64(self.current_y)):
            self.waypoints = np.delete(self.waypoints, self.point, axis=0)
        if np.round(self.waypoints[self.point][0]) == np.round(self.current_x) and np.round(self.waypoints[self.point][1] == np.round(self.current_y)):
            self.waypoints = np.delete(self.waypoints, self.point, axis=0)
        
        diff_x = [self.current_x - way_x for way_x in self.waypoints[:,0]]
        diff_y = [self.current_y - way_y for way_y in self.waypoints[:,1]]
        diff_dist = np.hypot(diff_x, diff_y)
        self.point = np.argmin(diff_dist)
        dist_old_ind = np.hypot((self.current_x - self.waypoints[self.point][0]),(self.current_y - self.waypoints[self.point][1]))
        if (self.point+1)<len(self.waypoints):
            dist_next_ind = np.hypot((self.current_x - self.waypoints[self.point+1][0]),(self.current_y - self.waypoints[self.point+1][1]))
        else:
            dist_next_ind = dist_old_ind
        if dist_next_ind < dist_old_ind:
            self.waypoints = np.delete(self.waypoints, self.point, axis=0)
            self.point += 1

        # ld = self.kd*self.v 
        ld = 1.5
        print("yaw: ", self.yaw)
        #if ld>closest point
        while ld > np.hypot((self.current_x - self.waypoints[self.point][0]),(self.current_y - self.waypoints[self.point][1])):
            if (self.point+1) < len(self.waypoints-1):
                self.waypoints = np.delete(self.waypoints, self.point, axis=0)
                self.point = self.point + 1
            else:
                break
        print("point: ", self.point) 
        alpha = np.arctan2(self.waypoints[self.point][1] - self.current_y, self.waypoints[self.point][0] - self.current_x) - np.round(self.yaw, 1)
        print("waypoints:", self.waypoints[self.point][0], self.waypoints[self.point][1])
        print("position:", self.current_x, self.current_y)
        print("alpha:", alpha)
        return alpha
    
    def steering(self):
        alpha = self.alpha()
        steering_angle = np.arctan2((2*self.L*np.sin(alpha)),(1.5))
        if steering_angle<-1:
            steering_angle = -1
        if steering_angle>1:
            steering_angle = 1
        print("steering_angle:",steering_angle)
        return steering_angle

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
        x_points=np.array([])
        y_points=np.array([])
        plot_waypoints = self.waypoints[:-1]
        
        while not rospy.is_shutdown():
            c = Control()
            if self.waypoints[self.point][0] == self.waypoints[-1][0] and self.waypoints[self.point][1] == self.waypoints[-1][1]:
                print("reached the destination")
                c.throttle = 0
                c.brake = 1
                self.controller.publish(c)
                break
            c.throttle, c.brake = self.vel_control()
            c.steer = self.steering()
            self.controller.publish(c)
            x_points=np.append(x_points,self.current_x)
            y_points=np.append(y_points,self.current_y)
            self.rate.sleep()
        print("x_current_plot:",x_points)
        print("x_plot:",plot_waypoints[:,0])
        plt.plot(plot_waypoints[:,0], plot_waypoints[:,1], label="Original Path")
        plt.plot(x_points, y_points, linestyle="--", label="Pure Pursuit Algorithm")
        plt.xlabel("x_coordinate")
        plt.ylabel("y_coordinate")
        plt.show()

if __name__ == "__main__":
    t = Pursuit()
    t.main()