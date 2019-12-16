#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import tf


def callback(odometry):
    global x
    global y
    global theta
    global Posex,Posey,Posetheta
    x=odometry.pose.pose.position.x
    y=odometry.pose.pose.position.y
    quat = odometry.pose.pose.orientation
    (angle1, angle2,theta) = euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
    (trans,rot)=listener.lookupTransform('/odom','/map',rospy.Time(0))
    Posex=x+trans[0]
    Posey=y+trans[1]
    Posetheta=theta+rot[3]
    (quat1, quat2,quat3,quat4) = quaternion_from_euler(angle1,angle2,Posetheta)
    data=Odometry()
    data.pose.pose.position.x=Posex
    data.pose.pose.position.y=Posey
    data.pose.pose.orientation=quat
    pose_publisher.publish(data)


rospy.init_node('pose_publisher', anonymous=True)
listener= tf.TransformListener()
pose_publisher= rospy.Publisher("/explo/pose",Odometry,queue_size=10) #was sign_list only
posSub = rospy.Subscriber('/odom',Odometry,callback)
print "Pose published"
rospy.spin()
