#! /usr/bin/env python
import rospy
import tf
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

def doPose(odom):
    #        
    broadcaster = tf2_ros.TransformBroadcaster()
    #       
    tfs = TransformStamped()
    tfs.header.frame_id = "odom"
    tfs.header.stamp = rospy.Time.now()
    tfs.child_frame_id = "base_link"
    tfs.transform.translation.x = odom.pose.pose.position.x
    tfs.transform.translation.y = odom.pose.pose.position.y
    tfs.transform.translation.z = 0.0
    tfs.transform.rotation.x = odom.pose.pose.orientation.x
    tfs.transform.rotation.y = odom.pose.pose.orientation.y
    tfs.transform.rotation.z = odom.pose.pose.orientation.z
    tfs.transform.rotation.w = odom.pose.pose.orientation.w
    #         
    broadcaster.sendTransform(tfs)

if __name__ == "__main__":

    rospy.init_node("odom_pub_tf")

    sub = rospy.Subscriber("/ep/odom",Odometry,doPose)

    rospy.spin()
