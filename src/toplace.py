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
goalx=0.0
goaly=0.0
def Robotpos(msg):
    global x
    global y
    global theta

    x= msg.pose.pose.position.x
    y= msg.pose.pose.position.y
    rot_q=msg.pose.pose.orientation #lees de positie en orientatie in van de odometry msg
    
    (roll, pitch, theta)=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
def listen(msg):
    global goalx
    global goaly
    
    goalx=msg.x
    goaly=msg.y


rclpy.init()

node = rclpy.create_node('odom_sub')   #subscriber en publisher aanmaken op Odom en cmd_vel
sub= node.create_subscription(
    Odometry, 'odom', Robotpos, 10
)
sub
sub2=node.create_subscription(Point, 'goalxy',listen, 10)
sub2
pub= node.create_publisher(Twist,"diff_cont/cmd_vel_unstamped", 10)
node.declare_parameter('x', value=0)

node.declare_parameter('y', value=0)



speed = Twist() 

 #maak een punt aan
#goal.x = 8.0 #float(node.get_parameter('x').value)

#goal.y = 0.0#float(node.get_parameter('y').value)

r= node.create_rate(4)
while rclpy.ok():
    rclpy.spin_once(node)  
    inc_x = goalx - x   #delta x
    inc_y = goaly - y   #delta y
    print(goalx,goaly)
    angle_to_goal = atan2(inc_y, inc_x)

    if abs(angle_to_goal - theta) > 0.1:   #robot naar punt laten draaien
        speed.linear.x=0.0
        speed.angular.z=0.2
    elif math.sqrt(inc_x*inc_x + inc_y*inc_y)<0.1: #stop als de robot bij het punt is
        speed.linear.x=0.0
        speed.angular.z=0.0
        pub.publish(speed)
        print("robot is op (" + str(x) +"," +str(y)+ ")")
        
    else:
        speed.linear.x=0.5    #als de robot gedraaid is rij naar voren
        speed.angular.z=0.0
        
    pub.publish(speed)  # stuur snelheid naar cmd_vel
    r.sleep