#!/usr/bin/env python3
import rclpy

from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import math
x= 0.0
y= 0.0
theta = 0.0
def Robotpos(msg):
    global x
    global y
    global theta

    x= msg.pose.pose.position.x
    y= msg.pose.pose.position.y
    rot_q=msg.pose.pose.orientation #lees de positie en orientatie in van de odometry msg
    
    (roll, pitch, theta)=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rclpy.init()

node = rclpy.create_node('odom_sub')   #subscriber en publisher aanmaken op Odom en cmd_vel
sub= node.create_subscription(
    Odometry, 'odom', Robotpos, 10
)
sub

pub= node.create_publisher(Twist,"cmd_vel", 10)


speed = Twist() 

goal = Point() #maak een punt aan
goal.x = -7.0
goal.y = 5.0

r= node.create_rate(4)
while rclpy.ok():
    rclpy.spin_once(node)  
    inc_x = goal.x- x   #delta x
    inc_y = goal.y - y   #delta y
    print(x,y)
    angle_to_goal = atan2(inc_y, inc_x)

    if abs(angle_to_goal - theta) > 0.1:   #robot naar punt laten draaien
        speed.linear.x=0.0
        speed.angular.z=0.3
    elif math.sqrt(inc_x*inc_x + inc_y*inc_y)<0.1: #stop als de robot bij het punt is
        speed.linear.x=0.0
        speed.angular.z=0.0
        pub.publish(speed)
        break
    else:
        speed.linear.x=0.5    #als de robot gedraaid is rij naar voren
        speed.angular.z=0.0
        
    pub.publish(speed)  # stuur snelheid naar cmd_vel
    r.sleep