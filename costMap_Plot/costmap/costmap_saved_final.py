#! /usr/bin/env python

import rospy
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist

from costmap.msg import Map_plot
from costmap.msg import Dijkstra

xRob = 0
yRob = 0
gridMap= [[],[]]
obstacles, unexplored_borders,dest = [[],[]],[[],[]],[0,0]

rospy.init_node('costmap_node')

pathPub = rospy.Publisher('/explo/path',Dijkstra,queue_size=1)
plotPub = rospy.Publisher('/explo/plot',Map_plot,queue_size = 1)
plotPathPub = rospy.Publisher('/explo/plot_path',Dijkstra,queue_size = 1)

def refreshPlot(isEnd):
    plot_data = Map_plot()
    plot_data.obs_x = obstacles[0]
    plot_data.border_x = unexplored_borders[0]
    plot_data.obs_y = obstacles[1]
    plot_data.border_y = unexplored_borders[1]
    plot_data.dest_x = [dest[0]]
    plot_data.dest_y = [dest[1]]
    plot_data.rob_x = [xRob]
    plot_data.rob_y = [yRob]
    plot_data.save = isEnd
    plotPub.publish(plot_data)

"""Convert an Array coordinate into x,y grid coordinate"""
def fromArrayToGrid(point,width,height,resolution):
    x = ((point)%width)
    y = ((point)/width)
    realx = x*resolution
    realy =  y*resolution
    return x,y,realx,realy

def getOdo(odo):
    global xRob, yRob
    xRob = odo.pose.pose.position.x
    yRob = odo.pose.pose.position.y

def read_cost(point):
    cost=costMap[point[0],point[1]]
    return cost

def set_cost(point,weight):
    costMap[point[0],point[1]]=weight

def read_cost_neighbour(point):#to have the minimum cost of the neighbors
    cost=[]
    neighbors=[[point[0],point[1]+1],[point[0]+1,point[1]],[point[0],point[1]-1],[point[0]-1,point[1]]]
    for i in neighbors:
        if read_cost(i)>=0:
            cost.append(read_cost(i))
    if len(cost)==0:
        return 10000
    else:
        return min(cost)

def get_min_neighbour(point):#to have the coordinates of minimum neighbor
    cost=[]
    neighbors=[[point[0],point[1]+1],[point[0]+1,point[1]],[point[0],point[1]-1],[point[0]-1,point[1]]]
    for i in neighbors:
        if not (i[0]<=0 or i[0]>=len(costMap[0])-1 or i[1]<=0 or i[1]>=len(costMap[1])-1):
            if read_cost(i)>=0:
                cost.append(read_cost(i))
            else:
                cost.append(10000)
        else:
            cost.append(10000)
        idx=np.argmin(cost)
        min_point=neighbors[idx]
    return min_point

def expansion(point):
    neighbors=[[point[0],point[1]+1],[point[0]+1,point[1]],[point[0],point[1]-1],[point[0]-1,point[1]]]
    for i in neighbors:
        if not (i[0]<=0 or i[0]>=len(costMap[0])-1 or i[1]<=0 or i[1]>=len(costMap[1])-1):
            if (read_cost(i)==-1 and not(gridMap[int(point[0]*cell_size),int(point[1]*cell_size)])==-1) or read_cost(i)>(read_cost(point)+1):
                set_cost(i,read_cost(point)+1)
                expansion(i)

def getMap(mapData):
    global xRob, yRob, obstacles, unexplored_borders,dest,gridMap, width, height, cell_size, costMap
    cell_size=6
    x,y,xtest,ytest,cost  = [],[],[],[],[]
    sampledx, sampledy = [],[]
    obstacles, unexplored_borders,dest = [[],[]],[[],[]],[0,0]
    costMap=[]
    map = mapData.data
    width = mapData.info.width
    height = mapData.info.height
    resolution = mapData.info.resolution # distance
    #print resolution, width, height, mapData.info.resolution, mapData.info.origin.position.x, mapData.info.origin.position.y
    gridMap = -1*np.ones((width,height))
    costMap = -1*np.ones((int(width/cell_size),int(height/cell_size)))

    for point in range(len(map)):
        xpos,ypos,realx,realy = fromArrayToGrid(point,width,height,resolution)
        gridMap[xpos,ypos] = map[point]
    for col in range(width):
        for row in range(height):
            if gridMap[row,col] >=25: # barrier
                obstacles[0].append((row-width/2)*resolution)
                obstacles[1].append((col-height/2)*resolution)
                costMap[int(row/cell_size),int(col/cell_size)]=-2

            if gridMap[row,col] != -1 and gridMap[row,col] < 50: # unknown cell
                if (gridMap[row+1,col] == -1 or gridMap[row,col+1] == -1 or gridMap[row-1,col] == -1 or gridMap[row,col-1] == -1):# and (row != xRob/resolution+width/2 and col != yRob/resolution+width/2):
                    unexplored_borders[0].append((row-width/2)*resolution) #unexplored meters
                    unexplored_borders[1].append((col-height/2)*resolution)  #unexplored meters
                    sampledx.append(int(row/cell_size))
                    sampledy.append(int(col/cell_size))
    firstx=int(xRob/resolution+width/2)
    firsty=int(yRob/resolution+height/2)
    startpoint=[int(firstx/cell_size),int(firsty/cell_size)] #probably uncorrect left for set_cost
    set_cost(startpoint,0)
    expansion(startpoint)
    costlist1=[]
    pathx,pathy,robpathx,robpathy, plotpathx, plotpathy,good_pointsx,good_pointsy=[],[],[],[],[],[],[],[]
    min_point1= []
    for i in range(len(sampledx)):
        if read_cost([sampledx[i],sampledy[i]])>=0:
            good_pointsx.append(sampledx[i])
            good_pointsy.append(sampledy[i])
    for i in range(len(good_pointsx)):
            if read_cost([good_pointsx[i],good_pointsy[i]])>=0:
                costlist1.append(read_cost([good_pointsx[i],good_pointsy[i]]))
    data = Dijkstra()
    if len(costlist1) > 0:
        idx=np.argmin(costlist1)
        min_point1=[good_pointsx[idx],good_pointsy[idx]]
        dest=min_point1
        current_point=min_point1
        dest[0] = min_point1[0]
        dest[1] = min_point1[1]
        pathx.append(current_point[0]*cell_size-width/2)
        pathy.append(current_point[1]*cell_size-height/2)
        robpathx.append((current_point[0]*cell_size-width/2)*resolution)
        robpathy.append((current_point[1]*cell_size-height/2)*resolution)
        plotpathx.append(current_point[0])
        plotpathy.append(current_point[1])
        for i in range(int(costlist1[idx])-1):
            pathx.append(get_min_neighbour(current_point)[0]*cell_size-width/2)
            pathy.append(get_min_neighbour(current_point)[1]*cell_size-height/2)
            robpathx.append((get_min_neighbour(current_point)[0]*cell_size-width/2)*resolution)
            robpathy.append((get_min_neighbour(current_point)[1]*cell_size-height/2)*resolution)
            plotpathx.append(get_min_neighbour(current_point)[0])
            plotpathy.append(get_min_neighbour(current_point)[1])
            current_point[0]=get_min_neighbour(current_point)[0]
            current_point[1]=get_min_neighbour(current_point)[1]
        data.xlist = robpathx
        data.ylist = robpathy
        data.numberDest = len(costlist1)
        isEnd = False
        pathPub.publish(data)
    else :
        print "no undiscovered places !!!"
        data.numberDest = len(costlist1)
        pathPub.publish(data)
        isEnd = True
    refreshPlot(isEnd)
    plotPath = Dijkstra()
    plotPath.xlist = robpathx
    plotPath.ylist = robpathy
    plotPathPub.publish(plotPath)
    plt.clf()
    ax = plt.gca()
    mat=(costMap.T)
    ax.matshow(mat)
    ax.plot(plotpathx,plotpathy,"w")
    ax.plot(sampledx,sampledy,"ro")
    ax.plot(dest[0],dest[1],"wo")
    ax.plot(good_pointsx,good_pointsy,"g.")
    ax.plot(startpoint[0],startpoint[1],"ro")
    plt.draw()

mapSub = rospy.Subscriber('/map',OccupancyGrid,getMap)
posSub = rospy.Subscriber('/explo/pose',Odometry,getOdo)#Create server that lead the navigation
fig = plt.figure()
plt.show()
plt.close(fig)
