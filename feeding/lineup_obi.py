import obi
import time
import argparse
import os.path as osp
from pathlib import Path

robot = obi.Obi('/dev/ttyUSB0')
print (robot.SerialIsOpen())

print (robot.VersionInfo())


robot.Wakeup()
time.sleep(17)

waypoint_file = "waypoints/successwaypoints.txt"

print('------------------------------------------------------------------------')
print('Press ENTER to send robot to feeding position')
user_input = input()
print('------------------------------------------------------------------------')

Waypoints = []
with open (waypoint_file) as WaypointFile:
    for InputLine in WaypointFile:
        IntWaypoint = list(map(int, InputLine.split(",")))
        Waypoints.append(IntWaypoint)
# print("Waypoints: ", Waypoints)
for WaypointIndex, Waypoint in enumerate(Waypoints):
    robot.SendOnTheFlyWaypointToObi(WaypointIndex, Waypoint)
robot.ExecuteOnTheFlyPath()

print('Press ENTER to proceed')
user_input = input()
print('------------------------------------------------------------------------')

robot.GoToSleep()
robot.Close()
print (robot.SerialIsOpen())

print ("all done")