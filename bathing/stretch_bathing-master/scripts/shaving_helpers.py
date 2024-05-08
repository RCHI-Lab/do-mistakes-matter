#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import threading
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import funmaptestcopy as fp

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

class ShavingHelperNode(hm.HelloNode):
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
        self.lift_position, self.lift_velocity, self.lift_effort = hm.get_lift_state(joint_states)
        self.wrist_contact_detector.update(joint_states,self.stop_the_robot_service)

    def move_arm_back(self):
        rospy.loginfo('Moving Arm Back')
        pose = {'gripper_aperture':.005,'wrist_extension': 0, 'joint_wrist_pitch':0, 'joint_wrist_yaw':2.5, 'joint_head_pan':-1.6, 'joint_head_tilt':-.5,'joint_lift':.68}
        self.move_to_pose(pose)
        rospy.sleep(3)
        self.grip()

    def move_to_shaving_position(self):
        rospy.loginfo('Moving to Shaving Position')
        pose = {'wrist_extension': .3, 'joint_wrist_pitch':-.3708, 'joint_wrist_yaw':.157, 'joint_lift':.8056, 'joint_gripper_finger_left':.1, 'joint_lift':.802}
        self.move_to_pose(pose)
        rospy.sleep(3)

    def arm_test_pos(self):
        rospy.loginfo('Moving to Arm Test Position')
        pose = {'wrist_extension':.2, 'joint_lift':.63}
        self.move_to_pose(pose)
        rospy.sleep(3)

    def arm_down_pos(self):
        rospy.loginfo('Moving to Arm Test Position')
        pose = {'joint_lift':.53}
        self.move_to_pose(pose)
        rospy.sleep(3)
    
    def grip(self):
        pose={'gripper_aperture':.075,'joint_wrist_roll':0}
        self.move_to_pose(pose)
        rospy.sleep(5)
        pose={'gripper_aperture':-.1}
        self.move_to_pose(pose)
        rospy.sleep(1)

    def contact_test(self, request):
        
        rospy.loginfo('Contact test')

        self.wrist_contact_detector.update(self.joint_states,self.stop_the_robot_service)
        success, message = self.wrist_contact_detector.move_until_contact('joint_lift',.3,-1,self.move_to_pose)
        return TriggerResponse(
            success = success,
            message = message 
        )

    def trigger_move_arm_back(self, request):

        self.move_arm_back()

        return TriggerResponse(
            success=True,
            message= 'Moved arm back!'
        )

    def trigger_shaving_position(self, request):
        self.move_to_shaving_position()

        return TriggerResponse(
            success=True,
            message= 'To Shaving Position!'
        )

    def trigger_arm_pos(self, request):
        self.arm_test_pos()

        return TriggerResponse(
            success=True,
            message='Moved to arm pos!'
        )


    def trigger_arm_pos_2(self, request):
        self.arm_down_pos()

        return TriggerResponse(
            success=True,
            message='Moved to arm pos!'
        )

    def trigger_grip(self, request):
        self.grip()

        return TriggerResponse(
            success=True,
            message = 'Grip complete'
        )

    def main(self):
        
        hm.HelloNode.main(self, 'shaving_helper', 'shaving_helper', wait_for_first_pointcloud = False)
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

        self.trigger_write_hello_service = rospy.Service('move_arm_back',
                                                         Trigger,
                                                         self.trigger_move_arm_back)

        self.trigger_write_hello_service = rospy.Service('shaving_position',
                                                         Trigger,
                                                         self.trigger_shaving_position)

        self.trigger_write_hello_service = rospy.Service('arm_pos',
                                                         Trigger,
                                                         self.trigger_arm_pos)

        self.trigger_write_hello_service = rospy.Service('arm_pos_2',
                                                         Trigger,
                                                         self.trigger_arm_pos_2)

        self.trigger_write_hello_service = rospy.Service('grip',
                                                         Trigger,
                                                         self.trigger_grip)

        self.trigger_write_hello_service = rospy.Service('contact_test',
                                                         Trigger,
                                                         self.contact_test)


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
    single_effort_threshold = 1.9
    av_effort_threshold = 1.5

    if (effort >= single_effort_threshold):
        rospy.loginfo('Pitch effort exceeded single_effort_threshold: {0} >= {1}'.format(effort,single_effort_threshold))
    if (av_effort >= av_effort_threshold):
        rospy.loginfo('Pitch average effort exceeded av_effort_threshold: {0} >= {1}'.format(av_effort,av_effort_threshold))

    return ((effort >= single_effort_threshold) or
            (av_effort >= av_effort_threshold))

if __name__ == '__main__':
    try:
        try:
            rospy.init_node('shaving_helper')

            listener=tf.TransformListener()
            node = ShavingHelperNode()
            node.main()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo('tf Exception')
    except KeyboardInterrupt:
        rospy.loginfo('Process Closing')