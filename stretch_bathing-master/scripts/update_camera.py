import pyrealsense2 as rs
from datetime import date
import numpy as np

if __name__ == '__main__':
    pipe = rs.pipeline()
    config = rs.config()
    config.enable_device('141722070195')
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color)
    pipe.start(config)

    #Image Capture
    for x in range(5):
        pipe.wait_for_frames()
    frameset=pipe.wait_for_frames()
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()

    print("Frames Captured")
    color = np.asanyarray(color_frame.get_data())
    colorizer = rs.colorizer()
    # Create alignment primitive with color as its target stream:
    align = rs.align(rs.stream.color)
    frameset = align.process(frameset)

    today = date.today()
    #Getting ply file
    ply = rs.save_to_ply("/home/hello-robot/catkin_ws/src/shaving_aruco/clouds/"+str(today)+".ply")
    ply.set_option(rs.save_to_ply.option_ply_binary, False)
    ply.set_option(rs.save_to_ply.option_ply_normals, True) 
    ply.process(colorizer.process(frameset))

    pipe.stop()