#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82


def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    return vel


# Main function
if __name__=="__main__":
    # Init node
    rospy.init_node('nodo_mover_cuadrado')

    # Publishers  // rospy.Publisher('topic_name', MessageType, queue_size=10)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Loop rate (in Hz)
    rate = rospy.Rate(10)

    twist = Twist()
    
    counter = 0

    linear_speed = 0.1
    angular_turn = 0.16
    while not rospy.is_shutdown():
        
        if counter == 00:
            twist.linear.x = checkLinearLimitVelocity(linear_speed); twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = checkLinearLimitVelocity(0)
        
        if counter == 100:
            twist.linear.x = checkLinearLimitVelocity(0.0); twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = checkLinearLimitVelocity(angular_turn)
        
        if counter == 200:
            twist.linear.x = checkLinearLimitVelocity(linear_speed); twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = checkLinearLimitVelocity(0)
        
        if counter == 300:
            twist.linear.x = checkLinearLimitVelocity(0.0); twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = checkLinearLimitVelocity(angular_turn)
        
        if counter == 400:
            twist.linear.x = checkLinearLimitVelocity(linear_speed); twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = checkLinearLimitVelocity(0)
        
        if counter == 500:
            twist.linear.x = checkLinearLimitVelocity(0.0); twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = checkLinearLimitVelocity(angular_turn)

        if counter == 600:
            twist.linear.x = checkLinearLimitVelocity(linear_speed); twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = checkLinearLimitVelocity(0)
        
        if counter == 700:
            twist.linear.x = checkLinearLimitVelocity(0.0); twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = checkLinearLimitVelocity(angular_turn)

        if counter == 800:
            twist.linear.x = checkLinearLimitVelocity(0.0); twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = checkLinearLimitVelocity(0.0)

        pub.publish(twist)        

        counter += 1
        rate.sleep()
    
    # finally:
    twist = Twist()
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    pub.publish(twist)