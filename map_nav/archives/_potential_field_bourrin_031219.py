#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from math import atan2,sqrt,cos,sin,acos,pi
from geometry_msgs.msg import PointStamped
import numpy as np
import time
from std_srvs.srv import Empty, EmptyResponse

epsilon= 10 #input("epsilon=?") attraction
etha=0.18 #input("etha=?") # repulsion
dist0=0.5 #input("distance of influence =?")
LaserValues=[]
Lasermem1=[]
Lasermem2=[]
Lasermem3=[]
Lasermin=0
x,y,theta=0,0,0
isRunning = False

rospy.init_node('assignement1')
posokp=rospy.Publisher('/reachPos_top',PointStamped,queue_size =1)
pospub=rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size =1)

def refresh_coordinates(odometry): #get actual position and orientation through the subscriber
    global x
    global y
    global theta
    x=odometry.pose.pose.position.x
    y=odometry.pose.pose.position.y
    quat = odometry.pose.pose.orientation
    (angle1, angle2,theta) = euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])

def getlaser(laser):
    global LaserValues
    global Lasermin
    global Laserangle
    LaserValues=laser.ranges
    Lasermin=laser.range_min
    Laserangle=laser.angle_increment

def ForceOfAtt(x,y,xgoal,ygoal):#calculate attraction force of point x y
    return np.array([-epsilon*(x-xgoal),-epsilon*(y-ygoal)])

def ForceOfRep(x,y,d,angle):#calculate repulsive force of point x y with d distance to obstacle (from laser scan)
    global dist0
    if d>dist0:
        return np.zeros(2)
    else:
        return np.array([(etha/(2*d**3))*d*cos(angle),(-etha/(2*d**3))*d*sin(angle)])

def sumForceOfRep(x,y):#calculate the sum of the repulsions of laser SCan points
    global LaserValues
    global Lasermem1
    global Lasermem2
    global Lasermem3
    global Laserangle
    S=np.zeros(2)
    for i in range(0,len(LaserValues)):
        if not (np.isnan(LaserValues[i])):# and LaserValues[i]>0.08:
            if i<len(LaserValues)/2:
                S+=ForceOfRep(x,y,LaserValues[i],theta+(len(LaserValues)/2-i)*Laserangle)
            else:
                S+=ForceOfRep(x,y,LaserValues[i],theta-(-len(LaserValues)/2+i)*(Laserangle))
        i+=1
    return S

def ForceMagnitude(Force): #calculate the magnitude of the force
    return sqrt(Force[0]**2+Force[1]**2)

def moveRobot(empty):
#    print Pose.header.frame_id
    timestmp = time.time()
    while time.time()-timestmp < 15:
        pass
    print "prout"
    response = EmptyResponse()
    return response

def NavPos(Pose):
    global isRunning
    if isRunning == True:
        return
    isRunning = True
    global Lasermem1,Lasermem2,LaserValues, theta, x,y
    speed=Twist()
    xgoal = Pose.point.x
    ygoal = Pose.point.y
    print "I am ",x,";",y
    print "On my way to ",xgoal,";",ygoal
    while sqrt(pow(xgoal-x,2)+pow(ygoal-y,2))>0.1: #we stop when less than 5cm away from goalpoint
        if Lasermin<dist0:
            Lasermem3=Lasermem2
            Lasermem2=Lasermem1
            Lasermem1=LaserValues

        Force=ForceOfAtt(x,y,xgoal,ygoal)-sumForceOfRep(x,y)
        endTheta=atan2(Force[1],Force[0])
        print Force
        print endTheta*180/pi
        while not((theta > (endTheta-0.15)) and (theta<(endTheta+0.15))) :     #aiming
            if theta < endTheta:
                speed.angular.z = 0.5
            else:
                speed.angular.z = -0.5
            speed.linear.x =0.0
            pospub.publish(speed)
        speed.angular.z = 0.0
        pospub.publish(speed)
        print endTheta*180/pi
        print theta*180/pi
        xStart = x
        yStart = y
        while sqrt(pow(y-yStart,2)+pow(x-xStart,2)) < 0.2:
            if sqrt(pow(xgoal-x,2)+pow(ygoal-y,2))<2:
                speed.linear.x=sqrt(pow(xgoal-x,2)+pow(ygoal-y,2))/15+0.03
            elif ForceMagnitude(Force)/(max(epsilon,etha))>0.15:
                speed.linear.x =0.15
            else:
                speed.linear.x = ForceMagnitude(Force)/(max(epsilon,etha))
            pospub.publish(speed)
    speed.linear.x = 0.0
    posOK = PointStamped()
    posOK.header.frame_id = "PoseReached"
    posOK.point.x = x
    posOK.point.y = y
    print "goal reached",x,";",y
    pospub.publish(speed)
    posokp.publish(posOK)
    isRunning = False


possub = rospy.Subscriber('/odom',Odometry,refresh_coordinates)
laserSub= rospy.Subscriber('/hokuyo_laser',LaserScan,getlaser)
reachPos = rospy.Subscriber('/reachPos_top',PointStamped,NavPos)
moveService = rospy.Service('/explo/moveRob',Empty,moveRobot)

rospy.spin()
