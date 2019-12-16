#! /usr/bin/env python

import rospy
import time
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.offsetbox import OffsetImage, AnnotationBbox

from costmap.msg import sign_message
from costmap.msg import Map_plot
from costmap.msg import Dijkstra

signs_array = [[],[],[],[]]
current_path = [[],[]]

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
    print "signs_array", signs_array

def refreshPath(path):
    global current_path
    current_path = [[],[]]
    current_path[0] = path.xlist
    current_path[1] = path.ylist

def create_legende():
    Unexp = mpatches.Patch(color='green', label='Unexplored fronteer')
    Obstacles = mpatches.Patch(color='black', label='obstacles')
    Robot_Position = mpatches.Patch(color='blue', label='Robot Position')
    Next_Pos = mpatches.Patch(color='red', label='Next Destination')
    return [Obstacles, Unexp, Robot_Position, Next_Pos]

def plotMap(data):
    plt.clf()
    ax = plt.gca()
    ax.plot(data.obs_x,data.obs_y,'k.')
    ax.plot(data.border_x,data.border_y,'g.')
    ax.plot(data.rob_x,data.rob_y,'bo')
#    ax.plot(data.dest_x,data.dest_y,'go')
    ax.plot(current_path[0],current_path[1],'r-')
#   ax.plot(current_path[0][len(current_path[0])-1],current_path[1][len(current_path[1])-1],'ro')
    for i in range(len(signs_array[0])):
        im = OffsetImage(plt.imread("/home/dacarte/icones/img{}.png".format(int(signs_array[0][i]))))
        ab = AnnotationBbox(im,(signs_array[1][i],signs_array[2][i]),frameon=False)
        ax.add_artist(ab)
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.title('Map')
    plt.axis("equal")
    legend = create_legende()
    plt.legend(handles=legend)
    plt.autoscale(enable=True,axis='both',tight=None)
    plt.draw()
    if data.save is True:
        plt.savefig("/home/dacarte/Map_images/map_"+mapTime+".png", bbox_inches='tight')

rospy.init_node('map_nav_node')
signSub = rospy.Subscriber('/explo/sign_list',sign_message,getSigns)
mapsub = rospy.Subscriber('/explo/plot',Map_plot,plotMap)
pathSub = rospy.Subscriber('/explo/plot_path',Dijkstra,refreshPath)
mapTime = str(int(time.time()))
fig = plt.figure()
plt.show()
plt.close(fig)
