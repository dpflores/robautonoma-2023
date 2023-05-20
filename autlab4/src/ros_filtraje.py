#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2 

from pcl_helper import *

from mesa_class import Cloud

class SuscriptorPointCloud(object):
    def __init__(self):
        topic = 'camera/depth_registered/points'
        self.sub = rospy.Subscriber(topic, PointCloud2, self.callback_point)
        self.pcl = ros2pcl(PointCloud2())
    def callback_point(self, msg):
        self.pcl = ros2pcl(msg)

    def get_pcl(self):
        return self.pcl
    

if __name__ == "__main__": # Inicio del programa principal

    rospy.init_node('nodo_pub_sub_pcl') # Inicializar el nodo
    sub = SuscriptorPointCloud() # Crear el suscriptor

    # Declarar del publicador 1
    topic1 = 'topico_mesa' 
    pub1 = rospy.Publisher(topic1, PointCloud2 , queue_size=1)

    # Declarar del publicador 2
    topic2 = 'topico_objetos' 
    pub2 = rospy.Publisher(topic2, PointCloud2 , queue_size=1)

    # Tiempo de ejecución del bucle (en Hz)
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        # Leer el valor actual (del suscriptor)
  
        pcl = sub.get_pcl()

        pcl1 = Cloud(pcl)
        pcl1.downsample(0.005)
        pcl1.pass_through( "z", 0.78, 1.87)
        pcl2 = Cloud(pcl1.cloud)
        pcl1.ransac(0.002, False)
        pcl2.ransac(0.003, True)
       

        pcl1_ros = pcl2ros(pcl1.cloud)
        pcl2_ros = pcl2ros(pcl2.cloud)

        pub1.publish(pcl1_ros)
        pub2.publish(pcl2_ros)
        
        # Esperar
        rate.sleep()