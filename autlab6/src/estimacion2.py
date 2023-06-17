#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import Imu
import numpy as np
from kalman import LinearFilter
from imuclass import SuscriptorImu
from imuclass import SuscriptorVel
    
if __name__ == "__main__":
    freq = 10
    x0 = np.array([[0, 0, 0, 0, 0, 0]]).T
    y0 = np.array([[0, 0, 0]]).T
    u0 = np.array([[0]]).T
    P0 = 0.01*np.eye(6)
    dt = 1/freq
    Fk = np.array([[1, 0, dt, 0, 0, 0],
                  [0, 1, 0, dt, 0, 0],
                  [0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 1, dt],
                  [0, 0, 0, 0, 0, 1]])
    Gk = np.array([[0]]).T
    Q = 10*np.eye(6)


    Hk = np.array([[0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 0, 1]])
    
    R = 10*np.eye(3)
    
    kalman = LinearFilter(x0,y0,u0,P0)
    imu = SuscriptorImu()
    vel = SuscriptorVel()

    br = tf.TransformBroadcaster()

    # Frecuencia del bucle
    rospy.init_node('nodo_kalman_estimation')
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():

        kalman.prediction_step(Fk,Gk,Q)
        kalman.yk = vel.get_vel()
        #print(kalman.yk)
        kalman.correction_step(Hk,R)

        print(kalman.xk)

        x = kalman.xk[0]
        y = kalman.xk[1]
        yaw = kalman.xk[4]

        br.sendTransform((x, y, 0.0),
                     tf.transformations.quaternion_from_euler(0, 0, yaw),
                     rospy.Time.now(),
                     "test_tf",
                     "odom")
        rate.sleep()