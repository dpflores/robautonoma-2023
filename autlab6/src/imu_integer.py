#!/usr/bin/env python

import rospy
import tf
import math
from sensor_msgs.msg import Imu
from imuclass import SuscriptorImu

# Variables globales
linear_velocity = [0.0, 0.0, 0.0]  # Velocidad lineal [vx, vy, vz]
position = [0.0, 0.0, 0.0]  # Posición [x, y, z]
previous_time = None
orientation = [0.0, 0.0, 0.0]  # Orientación en ángulos de Euler [roll, pitch, yaw]

# Inicializar broadcaster de TF
br = tf.TransformBroadcaster()


# Suscriptor IMU
imu = SuscriptorImu()



if __name__ == '__main__':
    rospy.init_node('nodo_imu_integer')
    # Loop rate (in Hz)
    rate = rospy.Rate(10)

    vx = 0.
    vy = 0.
    px = 0.
    py = 0.
    yaw = 0.

    
    # print("  - Z: {}".format(position[2]))
    while not rospy.is_shutdown():
        ax, ay, w = imu.get_value_inertial()

        vx += ax* 0.1
        vy += ay* 0.1
        
        px += vx*0.1
        py += vy*0.1
        yaw += w*0.1

        

        br.sendTransform((px, py, 0.0),
                     tf.transformations.quaternion_from_euler(0, 0, yaw),
                     rospy.Time.now(),
                     "test_tf",
                     "odom")
        print("Posición:")
        print("  - X: {}".format(px))
        print("  - Y: {}".format(py))

        rate.sleep()

