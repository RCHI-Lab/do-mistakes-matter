#!/usr/bin/env python

import rospy
import rosservice
import tf
import tf2_ros
import threading
import matplotlib.pyplot as plt
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import pickle

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from pid import PID

class LiftEffortNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.joint_states_lock = threading.Lock()
        self.joint_states=None
        self.time=[]
        self.val=[]

    def lift_pid(self):
        #initialize PD setpoint as first lift effort reported when the node is run
        lift = self.joint_states.name.index('joint_lift')
        wrist = self.joint_states.name.index('wrist_extension')
        lift_controller=PID(self.joint_states.effort[lift],5,.3,.2)
        wrist_controller=PID(self.joint_states.effort[wrist],5,.3,.2)
        curr_lift_effort=self.joint_states.effort[lift]
        curr_wrist_effort=self.joint_states.effort[wrist]
        while not rospy.is_shutdown():
            t=rospy.get_time()
            self.time.append(t)
            (new,change)=(lift_controller.update(t,curr_lift_effort))
            (wrist_new,wrist_change)=(wrist_controller.update(t,curr_wrist_effort))
            move_by=.001*change
            wrist_move_by=.001*wrist_change
            rospy.loginfo("Move lift by:" + str(move_by))
            rospy.loginfo("Move wrist by: "+str(wrist_move_by))
            """ lift_controller.setpoint=self.joint_states.effort[lift]
            wrist_controller.setpoint=self.joint_states.effort[wrist] """
            if .05<move_by<=.3 or -.2<=move_by<=-.05:
                new_pos=self.joint_states.position[lift]+move_by
                rospy.loginfo("moving lift from "+str(self.joint_states.position[lift])+" to "+str(new_pos))
                pose={'joint_lift':new_pos}
                self.move_to_pose(pose)
                rospy.sleep(2)
            if .05<abs(wrist_move_by)<=.1:
                pose={'wrist_extension':self.joint_states.position[wrist]+wrist_move_by}
                self.move_to_pose(pose)
                rospy.sleep(2)
            self.val.append(new)
            print(str((new,change)))
            curr_lift_effort=self.joint_states.effort[lift]
            curr_wrist_effort=self.joint_states.effort[wrist]

    def plot(self,time,val):
        fig=plt.figure(figsize=[15,7])
        ax1=fig.add_axes([0,0,1,1])
        ax1.plot(time,val,color='green')
        print(time)
        print(val)

    def joint_states_callback(self,joint_states):
        with self.joint_states_lock:
            self.joint_states=joint_states

    def trigger_lift_pid(self, request):
        self.lift_pid()
        pickle.dump((self.time,self.val),open('/home/hello-robot/catkin_ws/src/stretch_ros/shaving_aruco/tempdata/obs.pkl','wb'))

        return TriggerResponse(
            success=True,
            message = 'Reported lift effort'
        ) 

    def main(self):
        hm.HelloNode.main(self, 'effort_node', 'effort_node', wait_for_first_pointcloud = False)
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        self.trigger_write_hello_service = rospy.Service('monitor_effort',Trigger,self.trigger_lift_pid)

        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node("effort_node")
        node = LiftEffortNode()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('Process Closing')