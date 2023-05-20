#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2 

from pcl_helper import *



class SuscriptorPointCloud(object):
    def __init__(self):
        topic = 'camera/depth_registered/points'
        self.sub = rospy.Subscriber(topic, PointCloud2, self.callback_point)
        self.pcl = 0
    def callback_point(self, msg):
        self.pcl = ros2pcl(msg)

    def get_pcl(self):
        return self.pcl
    

if __name__ == "__main__": # Inicio del programa principal

    rospy.init_node('nodo_suscriptor_pcl') # Inicializar el nodo
    sub = SuscriptorPointCloud() # Crear el suscriptor

    

    # Tiempo de ejecución del bucle (en Hz)
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        # Leer el valor actual (del suscriptor)
  
        pcl = sub.get_pcl()

        

        print(pcl)
        
        # Esperar
        rate.sleep()