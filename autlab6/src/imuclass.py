#!/usr/bin/env python
# -*- coding: utf-8 -*-                                                                                             
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class SuscriptorImu(object):
    def __init__(self):
        topic = '/imu'
        self.pub = rospy.Subscriber(topic, Imu, self.callback)
        self.linear_vel = [0.0, 0.0]
        self.imu = Imu()
        
    def callback(self, msg):
        self.imu = msg

    def get_value_inertial(self):
        """
        Retorna la aceleracion lineal (en el sistema inercial u odom), y la velocidad angular

        """
        th = 2.0*np.arctan2(self.imu.orientation.z, self.imu.orientation.w)
        a = np.array([self.imu.linear_acceleration.x, self.imu.linear_acceleration.y, 0.0])
        R = np.array([[np.cos(th), np.sin(th), 0.0], [-np.sin(th), np.cos(th), 0.0], [0.0, 0.0, 1.0]])
        af = np.dot(R, a)  
        return af[0], af[1], self.imu.angular_velocity.z   # ax, ay, w
    
    def get_value_inertial_k(self):
        """
        Retorna la aceleracion lineal (en el sistema inercial u odom), y la velocidad angular

        """
        th = 2.0*np.arctan2(self.imu.orientation.z, self.imu.orientation.w)
        a = np.array([self.imu.linear_acceleration.x, self.imu.linear_acceleration.y, 0.0])
        R = np.array([[np.cos(th), np.sin(th), 0.0], [-np.sin(th), np.cos(th), 0.0], [0.0, 0.0, 1.0]])
        af = np.dot(R, a)  
        return np.array([[af[0], af[1], self.imu.angular_velocity.z]]).T

    def get_value_robot(self):
        """
        Retorna la aceleracion lineal (en el sistema local, del robot), y la velocidad angular

        """
        th = 2.0*np.arctan2(self.imu.orientation.z, self.imu.orientation.w)
        a = np.array([self.imu.linear_acceleration.x, self.imu.linear_acceleration.y, 0.0])
        R = np.array([[np.cos(2*th), np.sin(2*th), 0.0], [-np.sin(2*th), np.cos(2*th), 0.0], [0.0, 0.0, 1.0]])
        af = np.dot(R, a)  
        return af[0], af[1], self.imu.angular_velocity.z   # ax, ay, w

    def get_value_robot_k(self):
        """
        Retorna la aceleracion lineal (en el sistema local, del robot), y la velocidad angular

        """
        th = 2.0*np.arctan2(self.imu.orientation.z, self.imu.orientation.w)
        a = np.array([self.imu.linear_acceleration.x, self.imu.linear_acceleration.y, 0.0])
        R = np.array([[np.cos(2*th), np.sin(2*th), 0.0], [-np.sin(2*th), np.cos(2*th), 0.0], [0.0, 0.0, 1.0]])
        af = np.dot(R, a)  
        return np.array([[af[0], af[1], self.imu.angular_velocity.z]]).T


class SuscriptorVel(object):
    def __init__(self):
        topic = '/cmd_vel'
        self.pub = rospy.Subscriber(topic, Twist, self.callback)
        self.cmd_vel= Twist()

    def callback(self, msg):
        self.cmd_vel = msg

    def get_vel(self):
        return np.array([[self.cmd_vel.linear.x, self.cmd_vel.angular.z]]).T
        