#!/usr/bin/env python
import rospy
import tf
import numpy as np
import threading
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import funmaptestcopy as fp
import pickle
from datetime import date
import time
import sys

import error_randomization 

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

class BathingNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.trials=[]
        self.rate=2.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.wrist_contact_detector=fp.ContactDetector(get_wrist_pitch,pitch_contact_fn,move_increment=.02)
        self.wrist_contact_detector.get_lift_pos=get_lift_pose

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states
        self.lift_position, self.lift_velocity, self.lift_effort = hm.get_lift_state(joint_states)
        self.wrist_contact_detector.update(joint_states,self.stop_the_robot_service)

    def grip_helper(self):
        rospy.loginfo('Grip Helper')
        pose = {'gripper_aperture':0.07}
        self.move_to_pose(pose)
        rospy.sleep(5)
        pose = {'gripper_aperture':-0.1}
        self.move_to_pose(pose)
        rospy.sleep(1)

    def move_arm_back(self):
        rospy.loginfo('Moving Arm Back')
        pose = {'wrist_extension': .1, 'joint_wrist_pitch':0, 'joint_wrist_yaw': 0, 'joint_head_pan':-1.6, 'joint_head_tilt':-.5,'joint_lift':.80}
        self.move_to_pose(pose)
        rospy.sleep(3)

    def align_with_tag(self):
        rospy.loginfo('Aligning with Tag')
        try:
            listener.waitForTransform('bathing_start_tag','link_aruco_top_wrist',rospy.Time(0.0),rospy.Duration(1.0))
            (trans,rot)=listener.lookupTransform('bathing_start_tag','link_aruco_top_wrist',rospy.Time(0.0))
            rospy.loginfo("Bottom right to robot wrist: " + str((trans,rot)))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf exception")
        #trans = (y, x, z)
        wrist_extend_by = 0
        base_move_by = 0
        if trans[0]>.265:
            wrist_extend_by = trans[0]-.265
        if abs(trans[1])>0.01:
            base_move_by = -trans[1]
        elif trans[1]<-0.01:
            base_move_by = trans[1]
        pose = {'wrist_extension': wrist_extend_by, 'joint_wrist_pitch': 0, 'joint_wrist_yaw': 0, 'translate_mobile_base': base_move_by}
        self.move_to_pose(pose)
        rospy.sleep(1)

    def move_to_knee(self,msg):
        rospy.loginfo('Moving to Knee')
        today = date.today()
        rospy.loginfo(msg)
        ((kx, ky),(ax,ay)) = eval(msg.data)
        pose = {'translate_mobile_base':kx*(.01)}
        self.move_to_pose(pose)
        pose = {'wrist_extension': -ky*(.01)+.05}
        self.move_to_pose(pose)
        rospy.sleep(5)
        self.move_to_ankle(-1, 0, msg)

    def move_to_ankle(self, error_type, error_time, msg):
        rospy.loginfo('Moving to Ankle')
        today = date.today()
        rospy.loginfo(msg)
        ((kx, ky),(ax,ay)) = eval(msg.data)
        interval = (((ax)*(.01))/15.0)
        shift = .015
        p = self.joint_states.name.index('joint_wrist_pitch')
        #if error type 3, only move the base/wrist if before the time when error occurs
        #other types of error, call the function self.error at time
        
        (ind, error_type, error_time) = trials[self.trial_index]
        
        if error_type == 2:
            for i in range (0,error_time):
                pose = {'translate_mobile_base':interval}
                """ ,'wrist_extension':ky + shift} """
                self.move_to_pose(pose)
                current_effort_pitch = self.joint_states.effort[p]
                if (current_effort_pitch<0.75):
                    self.wrist_contact_detector.move_until_contact('joint_lift',.05,-1,self.move_to_pose)
                time.sleep(0.5)
                shift = shift*-1
        else:
            for i in range (0,15):
                if error_type>=0 and i >= error_time:
                    pose = self.error(error_type,msg)
                    pose['wrist_extension']+=shift
                else:
                    pose = {'translate_mobile_base':interval,'wrist_extension':-ky*.01 + shift}
                print("pose: " + str(pose))
                self.move_to_pose(pose)
                current_effort_pitch = self.joint_states.effort[p]
                if (current_effort_pitch<0.75):
                    self.wrist_contact_detector.move_until_contact('joint_lift',.05,-1,self.move_to_pose)
                time.sleep(0.5)
                shift = shift*-1
        rospy.sleep(1)
        self.trial_index+=1
        if (trial_index > len(self.trials)):
            print("Trials Complete")
            return
        else:
            print("Make sure the robot is aligned with the tag before you run the next trial)")
            cont = input("Ready to run the next trial? [y/n]: ")
            if (cont == 'y'):
                self.move_to_knee(msg)
            else:
                print("Ok. Exiting Now.")
                return

    def error(self,direction,msg):
        today = date.today()
        ((kx, ky),(ax,ay)) = eval(msg.data)
        interval = (ax/15.0)
        if direction < 1:
            pose = {'translate_mobile_base':interval*(.01),'wrist_extension':(-ky)*.01 + .085}
        else:
            pose = {'translate_mobile_base':interval*(.01),'wrist_extension':(-ky)*.01 - .085}
        return pose

    def trigger_grip(self, request):
        self.grip_helper()

        return TriggerResponse(
            success=True,
            message= 'Gripped Washcloth'
        )
    
    def trigger_move_arm_back(self, request):

        self.move_arm_back()

        return TriggerResponse(
            success=True,
            message= 'Moved arm back!'
        )

    def trigger_align_with_tag(self,request):
        self.align_with_tag()

        return TriggerResponse(
            success=True,
            message= 'Aligned With Tag'
        )

    def trigger_move_to_knee(self,request):
        self.move_to_knee()

        return TriggerResponse(
            success=True,
            message= 'Moved to Knee'
        )

    def trigger_move_to_ankle(self,request):
        self.move_to_ankle(1,7)

        return TriggerResponse(
            success=True,
            message= 'Moved to Ankle'
        )

    def main(self, trials):
        self.trials = trials
        self.trial_index = 0
        hm.HelloNode.main(self, 'bathing', 'bathing', wait_for_first_pointcloud = False)
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

        self.trigger_write_hello_service = rospy.Service('grip',
                                                         Trigger,
                                                         self.trigger_grip)

        self.trigger_write_hello_service = rospy.Service('move_arm_back',
                                                         Trigger,
                                                         self.trigger_move_arm_back)

        self.trigger_write_hello_service = rospy.Service('align_with_tag',
                                                         Trigger,
                                                         self.trigger_align_with_tag)

        """         self.trigger_write_hello_service = rospy.Service('move_to_knee',
                                                         Trigger,
                                                         self.trigger_move_to_knee)

        self.trigger_write_hello_service = rospy.Service('move_to_ankle',
                                                         Trigger,
                                                         self.trigger_move_to_ankle) """
        
        rate = rospy.Rate(self.rate)
        rospy.Subscriber("posepublisher", String, self.move_to_knee)
        
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
    single_effort_threshold = 1.3
    av_effort_threshold = 1

    if (effort >= single_effort_threshold):
        rospy.loginfo('Pitch effort exceeded single_effort_threshold: {0} >= {1}'.format(effort,single_effort_threshold))
    if (av_effort >= av_effort_threshold):
        rospy.loginfo('Pitch average effort exceeded av_effort_threshold: {0} >= {1}'.format(av_effort,av_effort_threshold))

    return ((effort >= single_effort_threshold) or
            (av_effort >= av_effort_threshold))

if __name__ == '__main__':
    try:
        try:
            #init trials to success
            trials = [(0, -1, 0)]
            study_trials = int(sys.argv[1])
            print(study_trials)
            if(study_trials):
                trials = error_randomization.generate_trials()
            print(trials)
        except:
            print("parameters passed incorrectly")
            
        try:
            rospy.init_node('bathing')
            listener=tf.TransformListener()
            node = BathingNode()
            node.main(trials)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo('tf Exception')
    except KeyboardInterrupt:
        rospy.loginfo('Process Closing')