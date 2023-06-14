#!/usr/bin/env python

import rospy
import tf

# Nombre del nodo
rospy.init_node('ejemplo_tf_broadcaster')

# Inicializar broadcaster de TF
br = tf.TransformBroadcaster()

# Posicion/orientacion inicial
x = 1.0; y = 1.0; theta = 3.1415/4.0

# Frecuencia del bucle
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    # Incremento de la posicion/orientacion
    x = x + 0.02
    y = y + 0.01
    theta = theta + 0.01
    # Hacer broadcast del tf: 'base_footprint' con respecto a 'odom'
    br.sendTransform((x, y, 0.0),
                     tf.transformations.quaternion_from_euler(0, 0, theta),
                     rospy.Time.now(),
                     "base_footprint",
                     "odom")
    rate.sleep()
