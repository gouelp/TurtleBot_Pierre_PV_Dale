#! /usr/bin/env python

"""
 Topics simu = /odom /map /cmd_vel_mux/input/teleop /hokuyo_laser
 Topics robot = /odom /map /cmd_vel_mux/input/teleop /scan
"""

import rospy
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from map_nav.srv import MoveRob, MoveRobRequest
from map_nav.msg import sign_message
from map_nav.msg import Map_plot

isCovered = False

xRob = 0.0
yRob = 0.0
thetaRob = 0.0
gridmap, costMap = [[],[]], [[],[]]
signs_array = [[],[],[],[]]
obstacles, unexplored_borders,dest = [[],[]],[[],[]],[0,0]

def main_function():
    reachPos = MoveRobRequest()
    reachPos.id = "360"
    res = execute_moove(reachPos)
    while isCovered is False:
        reachPos.id = "moveTo"
        reachPos.x = dest[0]
        reachPos.y = dest[1]
        res = execute_moove(reachPos)
        if res == "moved":
            print "Movement ok"
        elif res =="Timeout":
            print "Timeout error before pose reached"
    print "Finish, go too original position"
    reachPos.id = "moveTo"
    reachPos.x = 0.0
    reachPos.y = 0.0
    res = execute_moove(reachPos)
    refreshPlot()

def refreshPlot():
    plot_data = Map_plot()
    plot_data.obs_x = obstacles[0]
    plot_data.border_x = unexplored_borders[0]
    plot_data.obs_y = obstacles[1]
    plot_data.border_y = unexplored_borders[1]
    plot_data.dest_x = [dest[0]]
    plot_data.dest_y = [dest[1]]
    plot_data.rob_x = [xRob]
    plot_data.rob_y = [yRob]
    plot_data.save = isCovered
    plotPub.publish(plot_data)

def getOdo(odometry):
    global xRob, yRob, thetaRob
    xRob=odometry.pose.pose.position.x
    yRob=odometry.pose.pose.position.y
    quat = odometry.pose.pose.orientation
    (angle1, angle2,thetaRob) = euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])

def getSigns(signs):
    global signs_array
    signs_array = [[],[],[],[]]
    print "Sign Detected"
    for sign in range(len(signs.id_list)):
        signs_array[0].append(signs.id_list[sign])
        signs_array[1].append(signs.x_list[sign])
        signs_array[2].append(signs.y_list[sign])
        if signs.id_list[sign] == 0:
            signs_array[3].append('Dead worker') # danger
        elif signs.id_list[sign] == 1:
            signs_array[3].append('Alive worker')
        elif signs.id_list[sign] == 2:
            signs_array[3].append('Bio-hazard') # biohazard
        elif signs.id_list[sign] == 3:
            signs_array[3].append('Danger') # flammable
        elif signs.id_list[sign] == 5:
            signs_array[3].append('Flammable')
        elif signs.id_list[sign] == 6:
            signs_array[3].append('Smoke')
        elif signs.id_list[sign] == 7:
            signs_array[3].append('Toxic')
        elif signs.id_list[sign] == 8:
            signs_array[3].append('Radioactive')

"""Convert an Array coordinate into x,y grid coordinate"""
def fromArrayToGrid(point,width,height,resolution):
    x = point%width
    y = point/width
    realx = (x-width/2)*resolution
    realy = (y-height/2)*resolution
    return x,y,realx,realy

"""Convert x,y grid coordinate into a 1dim array coordinate"""
def fromGridToArray(x,y,width,height,resolution):
    point = (int(x)+(int(y))*width)
    return point

def fromRealToGrid(x,y,width,height,resolution):
    grid_x = x*resolution-width/2
    grid_y = y*resolution-height/2
    return int(math.ceil(grid_x)), int(math.ceil(grid_y))

def getMap(mapData):
    global mapSub, xRob, yRob, obstacles, unexplored_borders,dest
    mindest = 100000
    obstacles, unexplored_borders,dest = [[],[]],[[],[]],[0,0]
    map = mapData.data
    width = mapData.info.width
    height = mapData.info.height
    resolution = mapData.info.resolution
    print resolution, width, height, mapData.info.origin.position.x, mapData.info.origin.position.y
    gridmap = -1*np.ones((width,height))
    for point in range(len(map)):
        if map[point]!=-1:
            xpos,ypos,realx,realy = fromArrayToGrid(point,width,height,resolution)
            gridmap[xpos,ypos] = map[point]
            if map[point] >= 25:
                obstacles[0].append(realx)
                obstacles[1].append(realy)
            else:
                if map[fromGridToArray(xpos+1,ypos,width,height,resolution)] ==-1 or map[fromGridToArray(xpos,ypos+1,width,height,resolution)] == -1 or map[fromGridToArray(xpos-1,ypos,width,height,resolution)] == -1 or map[fromGridToArray(xpos,ypos-1,width,height,resolution)] ==-1:
                    dist = math.sqrt(math.pow(realx-xRob,2)+math.pow(realy-yRob,2))
                    if dist > 0.2 and realy < 3.0:
                        unexplored_borders[0].append(realx)
                        unexplored_borders[1].append(realy)
                        if dist<mindest and dist > 1.0 and realx > 0:
                            dest[0] = realx
                            dest[1] = realy
                            mindest = dist
    if len(unexplored_borders[0])== 0:
        isCovered = True
        print "Finis"
        dest[0] = 0.0
        dest[1] = 0.0
    refreshPlot()

rospy.init_node('map_nav_node')
execute_moove = rospy.ServiceProxy('/explo/moveRob',MoveRob)
rospy.wait_for_service('/explo/moveRob')
mapSub = rospy.Subscriber('/map',OccupancyGrid,getMap)
odoSub = rospy.Subscriber('/odom',Odometry,getOdo)
signSub = rospy.Subscriber('/sign_list',sign_message,getSigns)
plotPub = rospy.Publisher('explo/plot',Map_plot,queue_size = 1)
print "End Init"
main_function()

rospy.spin()  # Make the code run until closed
