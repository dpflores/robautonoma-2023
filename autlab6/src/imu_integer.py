#!/usr/bin/env python

import rospy
import tf
import math
from sensor_msgs.msg import Imu


# Variables globales
linear_velocity = [0.0, 0.0, 0.0]  # Velocidad lineal [vx, vy, vz]
position = [0.0, 0.0, 0.0]  # Posición [x, y, z]
previous_time = None
orientation = [0.0, 0.0, 0.0]  # Orientación en ángulos de Euler [roll, pitch, yaw]

# Inicializar broadcaster de TF
br = tf.TransformBroadcaster()

def imu_callback(data):
    global previous_time, linear_velocity, position, orientation

    if previous_time is None:
        previous_time = rospy.Time.now()
        return

    current_time = rospy.Time.now()
    dt = (current_time - previous_time).to_sec()  # Diferencia de tiempo en segundos

    # Integración de la aceleración para obtener la velocidad
    linear_velocity[0] += data.linear_acceleration.x * dt
    linear_velocity[1] += data.linear_acceleration.y * dt
    linear_velocity[2] += data.linear_acceleration.z * dt

    # Integración de la velocidad para obtener la posición
    position[0] += linear_velocity[0] * dt
    position[1] += linear_velocity[1] * dt
    position[2] += linear_velocity[2] * dt

    # Integración de las velocidades angulares para obtener los ángulos de orientación
    orientation[0] += data.angular_velocity.x * dt
    orientation[1] += data.angular_velocity.y * dt
    orientation[2] += data.angular_velocity.z * dt

    # Asegurarse de que los ángulos de orientación estén entre -180 y 180 grados
    orientation[0] = wrap_angle(orientation[0])
    orientation[1] = wrap_angle(orientation[1])
    orientation[2] = wrap_angle(orientation[2])


    # Actualizar el tiempo anterior
    previous_time = current_time

    # Imprimir los valores de velocidad y posición
    print("Aceleración:")
    print("  - X: {}".format(data.linear_acceleration.x))
    print("  - Y: {}".format(data.linear_acceleration.y))
    print("  - Z: {}".format(data.linear_acceleration.z))

    # Imprimir los valores de velocidad y posición
    # print("Velocidad:")
    # print("  - X: {}".format(linear_velocity[0]))
    # print("  - Y: {}".format(linear_velocity[1]))
    # print("  - Z: {}".format(linear_velocity[2]))

    # print("Posición:")
    # print("  - X: {}".format(position[0]))
    # print("  - Y: {}".format(position[1]))
    # print("  - Z: {}".format(position[2]))

    # print("Orientación (Ángulos de Euler):")
    # print("  - Roll:  {}".format(orientation[0]))
    # print("  - Pitch: {}".format(orientation[1]))
    # print("  - Yaw:   {}".format(orientation[2]))

    x = position[0]
    y = position[1]
    z = position[2]

    roll = orientation[0]
    pitch = orientation[1]
    yaw = orientation[2]
    br.sendTransform((x, y, 0.0),
                     tf.transformations.quaternion_from_euler(0, 0, yaw),
                     rospy.Time.now(),
                     "test_tf",
                     "base_footprint")

def wrap_angle(angle):
    # Asegurar que el ángulo esté entre -180 y 180 grados
    wrapped_angle = angle % (2*math.pi)
    if wrapped_angle > math.pi:
        wrapped_angle -= 2*math.pi
    return wrapped_angle

def imu_subscriber():
    rospy.init_node('imu_subscriber', anonymous=True)
    rospy.Subscriber("/imu", Imu, imu_callback)
    rospy.spin()

if __name__ == '__main__':
    
    imu_subscriber()

