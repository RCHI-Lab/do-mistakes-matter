#!/usr/bin/env python
import queue
import rospy
import tf
import numpy as np
import threading
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import funmaptestcopy as fp

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

class ExecuteTrajectoryNode(hm.HelloNode):
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

    def trajectory_callback(self, points_list):
        self.data = points_list.data
        self.points = np.reshape(self.data,(len(self.data)/3,3))

    def ready(self,request):
        pose={'joint_head_pan':-1.5,'joint_head_tilt':-.45,'joint_lift':.83,'joint_wrist_pitch':0,'joint_wrist_roll':0,'joint_wrist_yaw':0}
        self.move_to_pose(pose)
        rospy.sleep(1)
        """ pose={'wrist_extension':.1}
        self.move_to_pose(pose)
        rospy.sleep(1) """
        pose={'gripper_aperture':.075}
        self.move_to_pose(pose)
        rospy.sleep(5)
        pose={'gripper_aperture':-.1}
        self.move_to_pose(pose)
        rospy.sleep(1)
        return TriggerResponse(
            success=True,
            message='Completed Head Movement'
        )

    def trigger_trajec_execution(self, request):
        self.points = np.flip((self.points[self.points[:,0].argsort()]),0)
        rospy.loginfo("List of points: "+str(self.points))

        prev=self.points[0]
        moves=[]
        for pt in self.points:
            [xdiff,ydiff,zdiff] = pt-prev
            moves.append([xdiff,ydiff,zdiff])
            prev=pt
        rospy.loginfo(moves)
        for move in moves:
            with self.joint_states_lock:
                i = self.joint_states.name.index('joint_lift')
                j = self.joint_states.name.index('wrist_extension')
                current_pos_lift = self.joint_states.position[i]
                current_pos_wrist = self.joint_states.position[j]
            pose={'translate_mobile_base':-xdiff}
            self.move_to_pose(pose)
            rospy.sleep(.75)
            pose={'wrist_extension':current_pos_wrist-ydiff}
            self.move_to_pose(pose)
            rospy.sleep(.25)
            #move down until contact detected
            p = self.joint_states.name.index('joint_wrist_pitch')
            current_effort_pitch = self.joint_states.effort[p]
            if (current_effort_pitch<1.3):
                self.wrist_contact_detector.move_until_contact('joint_lift',.05,-1,self.move_to_pose)
            #rospy.loginfo(self.wrist_contact_detector.not_stopped())
            """ pose={'joint_lift':current_pos_lift+zdiff}
            self.move_to_pose(pose) """
            rospy.sleep(.25)
            """ try:
                listener.waitForTransform('link_shaver_head','sel_point',rospy.Time(0.0),rospy.Duration(1.0))
                (trans,rot)=listener.lookupTransform('link_shaver_head','sel_point',rospy.Time(0.0))
                rospy.loginfo("tag to robot wrist: " + str((trans,rot)))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("tf exception") """
        return TriggerResponse(
            success=True,
            message='Executed Trajectory'
        )

    def main(self):
        hm.HelloNode.main(self, 'exec_trajectory', 'exec_trajectory', wait_for_first_pointcloud = False)
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        self.trajectory_subscriber = rospy.Subscriber('/trajectory_points',Float64MultiArray, self.trajectory_callback)

        self.trigger_write_hello_service = rospy.Service('execute_trajectory',
                                                         Trigger,
                                                         self.trigger_trajec_execution)

        self.trigger_write_hello_service = rospy.Service('ready',
                                                         Trigger,
                                                         self.ready)

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
    single_effort_threshold = 1.2
    av_effort_threshold = .9

    if (effort >= single_effort_threshold):
        rospy.loginfo('Pitch effort exceeded single_effort_threshold: {0} >= {1}'.format(effort,single_effort_threshold))
    if (av_effort >= av_effort_threshold):
        rospy.loginfo('Pitch average effort exceeded av_effort_threshold: {0} >= {1}'.format(av_effort,av_effort_threshold))

    return ((effort >= single_effort_threshold) or
            (av_effort >= av_effort_threshold))

if __name__=='__main__':
    rospy.init_node("exec_trajectory")

    listener=tf.TransformListener()
    
    node = ExecuteTrajectoryNode()
    node.main()