import obi
import time
import argparse
import os.path as osp
from pathlib import Path
import error_randomization

parser = argparse.ArgumentParser(description='')
parser.add_argument('--subject-id', type=str, default='TEST', required=True)
parser.add_argument('--trial-idx', type=int, default=0)
args = parser.parse_args()

t0 = time.time()
# TODO ADD PRINTS FOR WHEN THE QUESTIONAIRRES SHOULD BE ASKED
print('========================================================================')
print(f'                          SUBJECT ID: {args.subject_id}')
print('========================================================================')

study_data_dir = '/home/kpputhuveetil/git/vBM-GNNdev/sasha_feeding/STUDY_DATA'
subject_dir = osp.join(study_data_dir, f'subject_{args.subject_id}')
config_file = osp.join(subject_dir, 'trial_config.ini')

Path(subject_dir).mkdir(parents=True, exist_ok=True)

if not osp.exists(config_file):
    trials = error_randomization.generate_trials()
    error_randomization.save_trials(trials, args.subject_id, subject_dir)
else:
    trials = error_randomization.read_trials(config_file)

robot = obi.Obi('/dev/ttyUSB0')
print (robot.SerialIsOpen())

print (robot.VersionInfo())


robot.Wakeup()
time.sleep(17)

for i in range(args.trial_idx, 9):
    
    sub_id = int(time.time())

    error_type = trials[i][1]
    trial_idx = trials[i][0]
    print('------------------------------------------------------------------------')
    print()
    print(f'Ready to start Trial {i} - config: {trials[i]}')
    print('Press ENTER to proceed')
    input()
    print('------------------------------------------------------------------------')

    direction = 0
    scoop_error_file = "waypoints/newscoopright.txt"
    if (error_type in [-1,2,3]):
        if (trial_idx < 5):
            scoop_error_file = "waypoints/newscoopright.txt"
        else: 
            scoop_error_file = "waypoints/newscoopleft.txt"
    else:
        if (error_type == 0):
            if (trial_idx < 5):
                scoop_error_file = "waypoints/scoopreallyshallow.txt"
            else:
                scoop_error_file = "waypoints/scoopshallowleft.txt"
        elif (error_type == 1):
            scoop_error_file = "waypoints/carrybowlerror.txt"


    Waypoints = []
    with open (scoop_error_file) as WaypointFile:
        for InputLine in WaypointFile:
            IntWaypoint = list(map(int, InputLine.split(",")))
            Waypoints.append(IntWaypoint)

    for WaypointIndex, Waypoint in enumerate(Waypoints):
        robot.SendOnTheFlyWaypointToObi(WaypointIndex, Waypoint)
    robot.ExecuteOnTheFlyPath()
    time.sleep(15)

    feeding_error_file = "waypoints/successwaypoints.txt"
    if (error_type in [-1,0,1]):
        feeding_error_file = "waypoints/successwaypoints.txt"
    else:
        if (error_type == 2):
            feeding_error_file = "waypoints/pitcherror.txt"
        else: #error_type = 3
            feeding_error_file = "waypoints/pitcherrorhigh.txt"
    print("feeding error file: ", feeding_error_file)

    Waypoints = []
    with open (feeding_error_file) as WaypointFile:
        for InputLine in WaypointFile:
            IntWaypoint = list(map(int, InputLine.split(",")))
            Waypoints.append(IntWaypoint)
    # print("Waypoints: ", Waypoints)
    for WaypointIndex, Waypoint in enumerate(Waypoints):
        robot.SendOnTheFlyWaypointToObi(WaypointIndex, Waypoint)
    robot.ExecuteOnTheFlyPath()
    time.sleep(15)

    clean_file = ""
    if (trial_idx < 5):
        clean_file = "waypoints/cleanwaypoints.txt"
    else:
        clean_file = "waypoints/leftcleanwaypoints.txt"
        
    Waypoints = []
    with open (clean_file) as WaypointFile:
        for InputLine in WaypointFile:
            IntWaypoint = list(map(int, InputLine.split(",")))
            Waypoints.append(IntWaypoint)

    for WaypointIndex, Waypoint in enumerate(Waypoints):
        robot.SendOnTheFlyWaypointToObi(WaypointIndex, Waypoint)
    robot.ExecuteOnTheFlyPath()
    time.sleep(16)  


robot.GoToSleep()
robot.Close()
print (robot.SerialIsOpen())

print ("all done")