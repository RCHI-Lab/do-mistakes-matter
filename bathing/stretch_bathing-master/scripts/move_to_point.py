#!/usr/bin/env python
import rospy
import tf
import numpy as np
import threading
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import funmaptestcopy as fp

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

class AlignWithTagNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate=2.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.wrist_contact_detector=fp.ContactDetector(get_wrist_pitch,pitch_contact_fn,move_increment=.02)
        self.wrist_contact_detector.get_lift_pos=get_lift_pose

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states
        self.wrist_contact_detector.update(self.joint_states,self.stop_the_robot_service)

    #Move the stretch to point on the body
    def trigger_move_to_point(self,request):
        try:
            listener.waitForTransform('shaver_aruco_test','link_aruco_top_wrist',rospy.Time(0.0),rospy.Duration(1.0))
            (trans,rot)=listener.lookupTransform('shaver_aruco_test','link_aruco_top_wrist',rospy.Time(0.0))
            rospy.loginfo("tag to robot wrist: " + str((trans,rot)))

            listener.waitForTransform('overhead_shaver_aruco_test','sel_point',rospy.Time(0.0),rospy.Duration(1.0))
            (trans2,rot2)=listener.lookupTransform('overhead_shaver_aruco_test','sel_point',rospy.Time(0.0))
            rospy.loginfo("tag to selected point: " + str((trans2,rot2)))
            self.finaltrans=[trans[0]+abs(trans2[0]),abs(trans[1])+trans2[1],abs(trans[2])-trans2[2]]
            rospy.loginfo("robot to point:" + str(self.finaltrans))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf exception")
        (x,y,z)=(self.finaltrans[0],self.finaltrans[1],self.finaltrans[2])
        with self.joint_states_lock:
            i = self.joint_states.name.index('joint_lift')
            j = self.joint_states.name.index('wrist_extension')
            current_pos_lift = self.joint_states.position[i]
            current_pos_wrist = self.joint_states.position[j]
        pose={'translate_mobile_base':abs(x)+.25}
        rospy.loginfo("x: "+str(pose))
        self.move_to_pose(pose)
        rospy.sleep(2.0)
        pose={'wrist_extension':current_pos_wrist+y-.02}
        rospy.loginfo("y: "+str(pose))
        self.move_to_pose(pose)
        rospy.sleep(2.0)
        p = self.joint_states.name.index('joint_wrist_pitch')
        current_effort_pitch = self.joint_states.effort[p]
        if (current_effort_pitch<.8):
                self.wrist_contact_detector.move_until_contact('joint_lift',.05,-1,self.move_to_pose)
        rospy.sleep(1.0)
        try:
            listener.waitForTransform('link_shaver_head','overhead_shaver_aruco_test',rospy.Time(0.0),rospy.Duration(1.0))
            (t1,r1)=listener.lookupTransform('link_shaver_head','overhead_shaver_aruco_test',rospy.Time(0.0))
            rospy.loginfo("shaver head to first point: " + str((t1,r1)))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf exception")
        return TriggerResponse(
            success=True,
            message='Completed Alignment w/ Tag'
        )
    
    def main(self):
        hm.HelloNode.main(self, 'get_robot_trans_to_tag', 'get_robot_trans_to_tag', wait_for_first_pointcloud = False)
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

        self.trigger_write_hello_service = rospy.Service('move_to_start_point',
                                                         Trigger,
                                                         self.trigger_move_to_point)
        
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

#referenced existing hello robot code for these in stretch_ros/funmap and stretch_ros/hello_misc.py
def get_wrist_pitch(joint_states):
    joint_name = 'joint_wrist_pitch'
    i = joint_states.name.index(joint_name)
    pitch_pos = joint_states.position[i]
    pitch_velocity = joint_states.velocity[i]
    pitch_effort = joint_states.effort[i]
    return [pitch_pos,pitch_velocity,pitch_effort]

def get_lift_pose(joint_states):
    joint_name = 'joint_lift'
    i = joint_states.name.index(joint_name)
    lift_pos = joint_states.position[i]
    return lift_pos

def pitch_contact_fn(effort, av_effort):
    single_effort_threshold = 1
    av_effort_threshold = 1.1

    """ if (effort >= single_effort_threshold):
        rospy.loginfo('Pitch effort exceeded single_effort_threshold: {0} >= {1}'.format(effort,single_effort_threshold))
    if (av_effort >= av_effort_threshold):
        rospy.loginfo('Pitch average effort exceeded av_effort_threshold: {0} >= {1}'.format(av_effort,av_effort_threshold)) """

    return ((effort >= single_effort_threshold) or
            (av_effort >= av_effort_threshold))

if __name__=='__main__':
    rospy.init_node("get_robot_trans_to_tag")

    listener=tf.TransformListener()
    
    node = AlignWithTagNode()
    node.main()