#!/usr/bin/env python
import rospy
import tf


if __name__=='__main__':
    rospy.init_node("overhead_trans_to_tag")

    listener=tf.TransformListener()
    try:
        listener.waitForTransform('overhead_shaver_aruco_test','sel_point',rospy.Time(0.0),rospy.Duration(1.0))
        (trans,rot)=listener.lookupTransform('overhead_shaver_aruco_test','sel_point',rospy.Time(0.0))
        rospy.loginfo((trans,rot))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.loginfo("tf exception")
    rospy.spin()