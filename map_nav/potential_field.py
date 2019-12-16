#! /usr/bin/env python

import rospy
import numpy as np
import time
from math import atan2,sqrt,cos,sin,acos,pi,pow
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty

from map_nav.srv import MoveRob, MoveRobResponse

isRobot = True

rotation_speed = 0.3
epsilon= 10 # Attraction Force
etha= 0.015 # Repulsion Force
dist0=0.6 # Distance of influence
noise_laser = 0.05 # Minimum usefull distance (remove noise)

LaserValues=[]
LaserStep,Laserangle, Lasermin = 0,0,30
x,y,theta=0,0,0

rospy.init_node('moving_node')
pospub=rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size =1)

def delay_s(delay):
    start_time = time.time()
    while time.time()-start_time < delay:
        pass

def refresh_coordinates(odometry):
    global x, y, theta
    x=odometry.pose.pose.position.x
    y=odometry.pose.pose.position.y
    quat = odometry.pose.pose.orientation
    (angle1, angle2,theta) = euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])

def getlaser(laser):
    global LaserValues, LaserStep, Laserangle
    LaserValues=laser.ranges
    LaserStep=laser.range_min
    Laserangle=laser.angle_increment

def ForceOfAtt(x,y,xgoal,ygoal):
    return np.array([-epsilon*(x-xgoal),-epsilon*(y-ygoal)])

def ForceOfRep(x,y,d,angle):
    global dist0, Lasermin
    if d>dist0 or d<noise_laser:
        return np.zeros(2)
    else:
        if d<Lasermin:
            Lasermin = d
        return np.array([(etha/(2*pow(d,3)))*d*cos(angle),-(etha/(2*pow(d,3)))*d*sin(angle)])

def sumForceOfRep(x,y):
    global LaserValues,Laserangle,Lasermin
    S=np.zeros(2)
    Lasermin = 30.0
    for i in range(0,len(LaserValues)):
        if not (np.isnan(LaserValues[i])):# and LaserValues[i]>0.08:
            if i<len(LaserValues)/2:
                S+=ForceOfRep(x,y,LaserValues[i],theta+(len(LaserValues)/2-i)*Laserangle)
            else:
                S+=ForceOfRep(x,y,LaserValues[i],theta-(-len(LaserValues)/2+i)*(Laserangle))
        i+=1
    return S

def ForceMagnitude(Force): #calculate the magnitude of the force
    return sqrt(pow(Force[0],2)+pow(Force[1],2))

def mesDiffAngle(target):
    diff = (target-theta)
    if diff>pi:
        diff-=2*pi
    elif diff <-pi:
        diff+=2*pi
    return diff

def rotateBy(angle):
    odoStart = theta
    speed = Twist()
    timer = time.time()
    speed.angular.z = rotation_speed
    while abs(mesDiffAngle(odoStart))>0.05 or time.time()-timer < 10:
        pospub.publish(speed)
        delay_s(0.05)
    speed.angular.z = 0.0
    pospub.publish(speed)

def go_backward(duration):
    print "I go back"
    speed = Twist()
    speed.linear.x = -0.3
    pospub.publish(speed)
    delay_s(duration)
    speed.linear.x = 0.0
    pospub.publish(speed)

def potentialField(xgoal, ygoal):
    global LaserValues, theta, x, y
    speed=Twist()
    timeout = time.time()
    refresh_map = timeout
    while sqrt(pow(xgoal-x,2)+pow(ygoal-y,2))>0.1: #we stop when less than 5cm away from goalpoint
        Force=ForceOfAtt(x,y,xgoal,ygoal)-sumForceOfRep(x,y)
        endTheta=atan2(Force[1],Force[0])
        diffAngle = mesDiffAngle(endTheta)
#        if abs(diffAngle)<0.05 and (Lasermin > 0.2 and Lasermin>0.05):
        if abs(diffAngle)<pi/4 and (Lasermin > 0.2 and Lasermin>0.05):
            speed.angular.z = 0.0
        else:
            speed.angular.z = diffAngle/pi
            if speed.angular.z > 0.3:
                speed.angular.z = 0.3
            elif speed.angular.z < -0.3:
                speed.angular.z = -0.3
            elif speed.angular.z > -0.2 and speed.angular.z < 0.0:
                speed.angular.z = -0.2
            elif speed.angular.z < 0.2 and speed.angular.z > 0.0:
                speed.angular.z = 0.2
        if abs(diffAngle)>1.5 or (Lasermin < 0.2 and Lasermin>0.035):
            speed.linear.x = 0.0
            speed.angular.z = 0.25
        elif sqrt(pow(xgoal-x,2)+pow(ygoal-y,2))<2:
            speed.linear.x=sqrt(pow(xgoal-x,2)+pow(ygoal-y,2))/15+0.03
        elif ForceMagnitude(Force)/(max(epsilon,etha))>0.15:
            speed.linear.x =0.15
        else:
            speed.linear.x = ForceMagnitude(Force)/(max(epsilon,etha))
        if speed.linear.x > 0.5:
            speed.linear.x = 0.5
        delay_s(0.4)
        pospub.publish(speed)
        if time.time()-refresh_map > 2:
            print "Force = ",Force
            print "Pose = [",x,";",y,"]"
            refresh_map = time.time()
#            mapPub.publish(Empty())
        if(time.time()-timeout) > 30:
            print "Timeout Stop"
            return -1
    speed.linear.x = 0.0
    speed.angular.z = 0.0
    print "Arrived in [",x,";",y,"]"
    pospub.publish(speed)
    return 1

def moveRobot(Pose):
    global x,y
    resp = MoveRobResponse()
    if Pose.id == '360':
        print "I'm turning"
        rotateBy(360)
        resp.result = "turned"
    elif Pose.id == 'moveTo' :
        print "From [",x,";",y,"] to [",Pose.x,";",Pose.y,"]"
        tag = 0
        for dist in LaserValues:
            if dist < 0.2 and dist>0.05:
                tag += 1
        if tag >0:
#            go_backward(3)
            pass
        res = potentialField(Pose.x,Pose.y)
#        rotateBy(360)
        if res == 1:
            resp.result = "moved"
        elif res == -1:
            resp.result = "Timeout"
    return resp

possub = rospy.Subscriber('/explo/pose',Odometry,refresh_coordinates)
mooveService = rospy.Service('/explo/moveRob',MoveRob,moveRobot)

if isRobot == True:
    laserSub= rospy.Subscriber('/scan',LaserScan,getlaser)
else:
    laserSub= rospy.Subscriber('/hokuyo_laser',LaserScan,getlaser)

rospy.spin()
