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

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from map_nav.srv import MoveRob, MoveRobRequest
from map_nav.msg import sign_message, Map_plot,Dijkstra

isCovered = False
wrongPath = True

xRob = 0.0
yRob = 0.0
path_x,path_y = [],[]
signs_array = [[],[],[],[]]

def main_function():
    reachPos = MoveRobRequest()
    plt_path = Dijkstra()
    reachPos.id = "360"
    res = execute_moove(reachPos)
    while isCovered is False:
        if wrongPath == False:
            newPathx = path_x[::-1]
            newPathy = path_y[::-1]
            plt_path.xlist = newPathx
            plt_path.ylist = newPathy
            pathPub.publish(plt_path)
            for i in range(len(newPathx)):
                reachPos.id = "moveTo"
                reachPos.x = newPathx[i]
                reachPos.y = newPathy[i]
                res = execute_moove(reachPos)
                if res == "moved":
                    print "Movement ok"
                elif res =="Timeout":
                    print "Timeout error before pose reached"
        reachPos.id = "360"
        res = execute_moove(reachPos)
    print "Finish, go to original position"
    reachPos.id = "moveTo"
    reachPos.x = 0.0
    reachPos.y = 0.0
    res = execute_moove(reachPos)

def getOdo(odometry):
    global xRob, yRob
    xRob=odometry.pose.pose.position.x
    yRob=odometry.pose.pose.position.y

def getSigns(signs):
    global signs_array
    signs_array = [[],[],[],[]]
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

def getPath(path):
    global path_x,path_y,isCovered, wrongPath
    if path.numberDest == -1 and len(signs_array[0]) < 4:
        wrongPath = True
    elif path.numberDest < 1 and len(signs_array[0]) >= 4:
        isCovered = True
    else:
        path_x = []
        path_y = []
        path_x = path.xlist
        path_y = path.ylist
        wrongPath = False

rospy.init_node('map_nav_node')
execute_moove = rospy.ServiceProxy('/explo/moveRob',MoveRob)
rospy.wait_for_service('/explo/moveRob')
odoSub = rospy.Subscriber('/odom',Odometry,getOdo)
signSub = rospy.Subscriber('/explo/sign_list',sign_message,getSigns)
pathSub = rospy.Subscriber('/explo/path',Dijkstra,getPath)
pathPub = rospy.Publisher('/explo/plot_path',Dijkstra,queue_size=1)
print "End Init"
main_function()

rospy.spin()  # Make the code run until closed
