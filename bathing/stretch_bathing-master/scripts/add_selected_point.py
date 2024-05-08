#!/usr/bin/env python

import rospy
import tf
import tf2_msgs.msg
import geometry_msgs.msg
import open3d as o3d
import networkx as nx
import numpy as np
from datetime import date
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

class PtAdder:

    def __init__(self,data):
        self.pub_tf = rospy.Publisher("/tf",tf2_msgs.msg.TFMessage, queue_size=1)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id="cam_1_color_optical_frame"
            t.child_frame_id="sel_point"
            t.header.stamp=rospy.Time.now()
            t.transform.translation.x=data[0]
            t.transform.translation.y=data[1]
            t.transform.translation.z=data[2]
            
            t.transform.rotation.x=0
            t.transform.rotation.y=0
            t.transform.rotation.z=0
            t.transform.rotation.w=1

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

if __name__ == '__main__':
    try:
        try:
            rospy.init_node('add_selected_point')
            listener=tf.TransformListener()
            
            today = date.today()
            pcd = o3d.io.read_point_cloud("/home/hello-robot/catkin_ws/src/shaving_aruco/clouds/"+str(today)+".ply")
            R = np.identity(3)  
            extent = np.ones(3)/.32 # trying to create a bounding box below 1 unit
            center = np.zeros(3) 
            obb = o3d.geometry.OrientedBoundingBox(center,R,extent)
            pcd = pcd.crop(obb)
            pcd = pcd.voxel_down_sample(voxel_size=0.012)
            vis = o3d.visualization.VisualizerWithEditing()
            vis.create_window()
            vis.add_geometry(pcd)
            vis.run()  
            vis.destroy_window()
            pdata = vis.get_picked_points()

            (x,y,z)=pcd.points[pdata[0]]
            (x,y,z)=(-x,-y,-z)

            G = nx.Graph()

            pcd_tree = o3d.geometry.KDTreeFlann(pcd)
            count=0
            for vertex in pcd.points:
                [k, idx, _] = pcd_tree.search_knn_vector_3d(vertex, 5)
                for v_curr in idx:
                    current_vertex=pcd.points[v_curr]
                    if not (count==v_curr):
                        val = np.asarray([vertex[0]-current_vertex[0],vertex[1]-current_vertex[1],vertex[2]-current_vertex[2]])
                        dist = np.sqrt(np.sum((val)**2, axis=0))
                        G.add_edge(count,v_curr,weight=dist)
                count=count+1
            search=(nx.bfs_tree(G,pdata[0]))
            path=nx.shortest_path(search,source=pdata[0],target=pdata[1])
            foundpcd = pcd.select_down_sample(path)
            foundpcd.paint_uniform_color([1, 0, 0])
            pcd.paint_uniform_color([0,1,0])

            pub=rospy.Publisher('trajectory_points',Float64MultiArray,queue_size=10)
            a=Float64MultiArray()
            d=np.array(foundpcd.points)
            d=np.ndarray.tolist((d.flatten()))
            a.data=d
            a.layout.data_offset = 0
            dim = []
            dim.append(MultiArrayDimension("points",len(d)/3,len(d)))
            dim.append(MultiArrayDimension("coords",3,1))
            a.layout.dim=dim
            rospy.loginfo(a)
            rospy.sleep(1)
            pub.publish(a)

            o3d.visualization.draw_geometries([pcd,foundpcd]) 
            

            temp=PtAdder([x,y,z])
            rospy.spin()
            #rospy.sleep(2)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf error")
    except KeyboardInterrupt:
        rospy.loginfo("closing")