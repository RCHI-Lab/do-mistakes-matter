#!/usr/bin/env python

import rospy
import rosservice
import tf
import tf2_ros
import threading
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from pid import PID

#Based on Kavya's stretch-bedding-manipulation code for detecting aruco markers, as well as the
#stretch_demo code, specifically the 'hello_world' node
class MoveToTagNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate=2.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states


    def move_to_tag_location(self,trans,rot):
        rospy.loginfo('Moving to Tag Height')
        with self.joint_states_lock:
            i = self.joint_states.name.index('joint_lift')
            j = self.joint_states.name.index('wrist_extension')
            current_pos_lift = self.joint_states.position[i]
            current_pos_wrist = self.joint_states.position[j]
        pose = {'joint_lift': current_pos_lift + (trans)[2],
                'wrist_extension': current_pos_wrist + abs((trans)[1]) - .25}
        rospy.loginfo('current lift position '+str(current_pos_lift))
        rospy.loginfo('move lift by '+str((trans)[2]))
        rospy.loginfo('current wrist position '+str(current_pos_wrist))
        rospy.loginfo('move wrist by '+str((trans)[1]))

        self.move_to_pose(pose)
        rospy.sleep(1.0)
        
    def find_next_tag(self, tagname):
        rospy.loginfo('Moving to the Next Tag')

        base_move_alpha=0.07
        total=base_move_alpha
        next_tag_found = False

        while not next_tag_found:
            pose = {'translate_mobile_base':base_move_alpha}
            self.move_to_pose(pose)
            rospy.sleep(7)
            looktime = rospy.Time.now()
            rospy.sleep(1)

            try:
                listener.waitForTransform('link_wrist_yaw_bottom',tagname,looktime,rospy.Duration(1.0))
                (t_new,r_new)=listener.lookupTransform('link_wrist_yaw_bottom',tagname,looktime)
                #To IMPLEMENT: break only if the x distance between wrist/tag is within a certain threshold (e.g, near enough to reach the tag)
                print("--Wrist Transformation to Newly Detected Tag--")
                self.trans_new=t_new
                self.rot_new=r_new
                print(self.trans_new,self.rot_new)
                if abs((self.trans_new)[2])<=.06:
                    print("next tag found)")
                    self.move_to_tag_location(self.trans_new,self.rot_new)
                    next_tag_found = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
                continue
            print("Total: "+str(total))
            if total>.55:
                print("test")
                exit()
            total=total+base_move_alpha
            

    def lift_height_test(self):
        rospy.loginfo('Lift Height Test')
        pose = {'joint_lift':.802}
        self.move_to_pose(pose)
        rospy.sleep(1)

    def trigger_move_to_height(self, request):
        
        self.move_arm_back()

        listener.waitForTransform('link_wrist_yaw_bottom','shaver_aruco_test',rospy.Time(0.0),rospy.Duration(1.0))
        (self.trans,self.rot)=listener.lookupTransform('link_wrist_yaw_bottom','shaver_aruco_test',rospy.Time(0.0))
        print("--Wrist Transformation--")
        print(self.trans,self.rot)

        self.move_to_tag_location(self.trans,self.rot)
        rospy.sleep(5)
        self.move_arm_back()
        self.find_next_tag('shaver_aruco_test_2')
        rospy.sleep(3)
        self.move_arm_back()

        return TriggerResponse(
            success=True,
            message='Completed Movement :) Remember to restart the detect_shaving_marker_node!'
        )
    """ 
    def trigger_report_lift_effort(self, request):
        self.lift_pid()

        return TriggerResponse(
            success=True,
            message = 'Reported lift effort'
        ) """


    def main(self):
        hm.HelloNode.main(self, 'detect_shaving_marker', 'detect_shaving_marker', wait_for_first_pointcloud = False)

        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        
        self.trigger_write_hello_service = rospy.Service('move_to_height',
                                                         Trigger,
                                                         self.trigger_move_to_height)

        """ self.trigger_write_hello_service = rospy.Service('report_lift_effort',
                                                         Trigger,
                                                         self.trigger_report_lift_effort) """
 
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        try:
            rospy.init_node('detect_shaving_marker')

            listener=tf.TransformListener()
            node = MoveToTagNode()
            node.main()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo('tf Exception')
    except KeyboardInterrupt:
        rospy.loginfo('Process Closing')
