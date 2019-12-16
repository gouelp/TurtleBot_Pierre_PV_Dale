#!/usr/bin/env python

import roslib
roslib.load_manifest('turtleassignement')
import rospy
import cv2
import numpy as np
from std_msgs.msg import *
from sensor_msgs.msg import Image
from turtleassignement.msg import  sign_message
from cv_bridge import CvBridge, CvBridgeError
from math import sqrt,pi,cos,sin
from tf.transformations import euler_from_quaternion
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry



def get_image(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
def signx(x,y,theta,dx,dy):
    return x+dx*cos(theta)-dy*sin(theta)
def signy(x,y,theta,dx,dy):
    return y+dy*cos(theta)+dx*sin(theta)
def refresh_coordinates(odometry): #get actual position and orientation through the subscriber
    global x
    global y
    global theta
    x=odometry.pose.pose.position.x
    y=odometry.pose.pose.position.y
    quat = odometry.pose.pose.orientation
    (angle1, angle2,theta) = euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
def callback(data):
    global min_dist
    min_dist=0.1
    if not (len(data.data)==0):

        id=data.data[0]

        (trans,rot)=listener.lookupTransform('/odom','/object_%d'%id,rospy.Time(0)) #map was /base_footprint
        (trans1,rot1)=listener.lookupTransform('/map','/odom',rospy.Time(0))
        # signx=trans[0]+trans1[0]
        signy=trans[1]+trans1[1]
        print("sign found !")

        #worker identification part:
        if id==1:
            print('worker sign detected, identifying...')
            blur=cv2.GaussianBlur(cv_image,(5,5),1)
            gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
            #cv2.imshow("gray",gray)
            ret,tresh = cv2.threshold(gray,100,255,cv2.THRESH_BINARY_INV)
            #cv2.imshow("tresh",tresh)
            img_dilation = cv2.dilate(tresh, kernel, iterations=6)
            #cv2.imshow("img_dilation",img_dilation)
            img_erosion = cv2.erode(img_dilation, kernel, iterations=6)
            #cv2.imshow("img_erosion",img_erosion)
            edges = cv2.Canny(img_erosion,10,200)
            #cv2.imshow("canny",edges)
            detected_circles = cv2.HoughCircles(edges,cv2.HOUGH_GRADIENT,0.2,10,param1=100,param2=30,minRadius=1,maxRadius=1000)
            if detected_circles is not None:
                circle = np.uint16(np.around(detected_circles))
                for i in circle[0,:]:
                    cv2.circle(edges,(i[0],i[1]),i[2],(255,0,255),2)
                    cv2.circle(edges,(i[0],i[1]),2,(255,0,255),2)
            idx = (np.abs(circle[0,:][2])).argmax()
            cX=circle[0,:][idx][0]
            cY=circle[0,:][idx][1]
            radius=circle[0,:][idx][2]
            mask = np.zeros(cv_image.shape[:2], np.uint8)
            cv2.circle(mask, (cX, cY), int(radius), (255, 255, 255), cv2.FILLED)
            #cv2.imshow('mask',mask)
            average=cv2.mean(cv_image,mask)
            if average[1]>average[2]:

                print ('alive worker to rescue !')
                #cv2.destroyAllWindows()

                check=False
                for i in list_of_signs:
                    if ((i[1]-min_dist)<trans[0]<(i[1]+min_dist) and (i[2]-min_dist)<trans[1]<(i[2]+min_dist)):
                        check=True

                if check==False:
                    list_of_signs.append([data.data[0],trans[0],trans[1]])
                    x_list.append(trans[0])
                    y_list.append(trans[1])
                    id_list.append(id)
                    check=True
                    sign=sign_message()
                    sign.id_list=id_list
                    sign.x_list=x_list
                    sign.y_list=y_list
                    sign_publisher.publish(sign)
                if init in list_of_signs:
                        list_of_signs.remove(init)
            elif average[2]>average[1]:

                print ('unfortunately this worker is dead...')
                #cv2.destroyAllWindows()

                check=False
                for i in list_of_signs:
                    if (i[1]-min_dist)<(trans[0])<(i[1]+min_dist) and (i[2]-min_dist)<(trans[1])<(i[2]+min_dist):
                        check=True
                if check==False:
                    list_of_signs.append([0,trans[0],trans[1]])
                    x_list.append(trans[0])
                    y_list.append(trans[1])
                    id_list.append(0)
                    check=True
                    sign=sign_message()
                    sign.id_list=id_list
                    sign.x_list=x_list
                    sign.y_list=y_list
                    sign_publisher.publish(sign)
                if init in list_of_signs:
                        list_of_signs.remove(init)

            cv2.waitKey(200)
        else:
            cv2.destroyAllWindows()

            check=False
            for i in list_of_signs:
                if (i[1]-min_dist)<(trans[0])<(i[1]+min_dist) and (i[2]-min_dist)<trans[1]<(i[2]+min_dist):
                    check=True
            if check==False:
                list_of_signs.append([data.data[0],trans[0],trans[1]])
                x_list.append(trans[0])
                y_list.append(trans[1])
                id_list.append(id)
                check=True
                sign=sign_message()
                sign.id_list=id_list
                sign.x_list=x_list
                sign.y_list=y_list
                sign_publisher.publish(sign)
            if init in list_of_signs:
                    list_of_signs.remove(init)







    print("list_of_signs",list_of_signs)

if __name__ == '__main__':
    rospy.init_node('sign_finder', anonymous=True)

    listener= tf.TransformListener()
    sign_publisher= rospy.Publisher("/explo/sign_list",sign_message,queue_size=10) #was sign_list only
    object_sub = rospy.Subscriber("/objects",Float32MultiArray,callback)#topic from findobject2d
    possub = rospy.Subscriber('/explo/pose',Odometry,refresh_coordinates)
    image_sub = rospy.Subscriber("/image_topic_compressed",Image,get_image) #CHAGE between simu and real
    bridge = CvBridge()
    id_list=[]
    x_list=[]
    y_list=[]
    init=[-1,-1,-1]
    kernel = np.ones((3,3),np.uint8)
    list_of_signs=[[-1,-1,-1]]

    try:
        rospy.spin()


    except KeyboardInterrupt:

        print("Shutting down")
