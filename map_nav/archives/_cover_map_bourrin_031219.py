#! /usr/bin/env python

import rospy
import time
import math
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist

xRob = 0
yRob = 0
thetaRob = 0
refresh_map = True
isRotating = True

rospy.init_node('map_nav_node')
reachPosPus = rospy.Publisher('/reachPos_top',PointStamped,queue_size=1)
pospub=rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size =1)

"""Convert an Array coordinate into x,y grid coordinate"""
def fromArrayToGrid(point,width,height,resolution):
    x = ((point%width)-width/2)
    y = ((point/width)-height/2)
    realx = x*resolution
    realy =  y*resolution
    return x,y,realx,realy

"""Convert x,y grid coordinate into a 1dim array coordinate"""
def fromGridToArray(x,y,width,height,resolution):
    point = (int(x)+height/2+(int(y)+height/2)*width)
    return point

def getOdo(odo):
    global xRob, yRob, thetaRob
    quat = odo.pose.pose.orientation
    xRob = odo.pose.pose.position.x
    yRob = odo.pose.pose.position.y
    #Get the angle from the quaternion given by odometry
    (angle1, angle2,thetaRob) = euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])

def getNavStatus(status):
    global refresh_map
    if status.header.frame_id == "PoseReached":
        refresh_map = True

def doFullRotation():
    global isRotating
    isRotating = True
    odoStart = thetaRob
    speed = Twist()
    timer = time.time()
    speed.angular.z = 0.3
    while math.sqrt(math.pow(odoStart-thetaRob,2))>0.1 or time.time()-timer < 10:
        pospub.publish(speed)
        pass
    speed.angular.z = 0.0
    pospub.publish(speed)
    isRotating = False
    print "Finis"

"""Simple map analysis function
    - Get a map from /map topic (mapserver or gmapping)
    - For every point discoverd and non-Empty
        - Get x,y coordinate to plot the point
        - Convert back into array coordinate
        - Get x,y coordinate from converted array point
        - Plot the new point into another figure
"""
def getMap(mapData):
    global xRob, yRob, refresh_map
    if isRotating == True:
        return
    x,y,xtest,ytest = [],[],[],[]
    unexploredx, unexploredy = [],[]
    destx, desty = None ,None
    mindest = 100000
    reachPos = PointStamped()
    reachPos.header.frame_id = "reachMinFront"

    if refresh_map:
        refresh_map = False
        map = mapData.data
        width = mapData.info.width
        height = mapData.info.height
        resolution = mapData.info.resolution
        print resolution, width, height, mapData.info.resolution, mapData.info.origin.position.x, mapData.info.origin.position.y

        timeStamp = time.time()
        for point in range(len(map)):
            if map[point]!=-1:
                xpos,ypos,realx,realy = fromArrayToGrid(point,width,height,resolution)
                indic = 0
                for xp in range(-1,1):
                    for yp in range(-1,1):
                        if map[fromGridToArray(xpos+xp,ypos+yp,width,height,resolution)] == -1:
                            indic +=1
                if indic <3:
                    if map[point] >= 50:
                        x.append(realx)
                        y.append(realy)
                    else:
                        if map[fromGridToArray(xpos+1,ypos,width,height,resolution)] ==-1 or map[fromGridToArray(xpos,ypos+1,width,height,resolution)] == -1 or map[fromGridToArray(xpos-1,ypos,width,height,resolution)] == -1 or map[fromGridToArray(xpos,ypos-1,width,height,resolution)] ==-1:
                            unexploredx.append(realx)
                            unexploredy.append(realy)
                            dist = math.sqrt(math.pow(realx-xRob,2)+math.pow(realy-yRob,2))
                            if dist<mindest:
                                destx = realx
                                desty = realy
                                mindest = dist
        if(destx is not None):
            reachPos.point.x = destx
            reachPos.point.y = desty
            print "GO: ",destx,";",desty
            print "I'm in ",xRob,";",yRob
            reachPosPus.publish(reachPos)
        timeStamp = time.time()-timeStamp
        print "fini en ",timeStamp,' s'
        plt.figure(0)
        plt.plot(x,y,'b.',label="map")
        plt.plot(unexploredx,unexploredy,'r.',label="unexplored border")
        plt.plot([xRob],[yRob],'ko',label="Robot")
        plt.plot([destx],[desty],'go',label="First Destination")
        plt.xlabel('x position')
        plt.ylabel('yposition')
        plt.legend()
        plt.title('Map Graph')
        plt.show()

mapSub = rospy.Subscriber('/map',OccupancyGrid,getMap)
posSub = rospy.Subscriber('/odom',Odometry,getOdo)#Create server that lead the navigation
isPosReached = rospy.Subscriber('/reachPos_top',PointStamped,getNavStatus)

doFullRotation()

rospy.spin() #make the code run until closed
