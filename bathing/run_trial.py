import subprocess
import argparse
import os.path as osp
from pathlib import Path
import time
import sys
import error_randomization


parser = argparse.ArgumentParser(description='')
parser.add_argument('--subject-id', type=str, default='TEST', required=True)
parser.add_argument('--trial-idx', type=int, default=0)
args = parser.parse_args()

t0 = time.time()

print('========================================================================')
print(f'                          SUBJECT ID: {args.subject_id}')
print('========================================================================')

study_data_dir = '/home/kpputhuveetil/git/vBM-GNNdev/sasha_bed_bathing/STUDY_DATA'

subject_dir = osp.join(study_data_dir, f'subject_{args.subject_id}')
config_file = osp.join(subject_dir, 'trial_config.ini')

Path(subject_dir).mkdir(parents=True, exist_ok=True)

# * only run a new trial order randomization if there is no existing trial config file
if not osp.exists(config_file):
    trials = error_randomization.generate_trials()
    error_randomization.save_trials(trials, args.subject_id, subject_dir)
else:
    trials = error_randomization.read_trials(config_file)

print('Full Trial List:', trials)
print(f'Starting with Trial {args.trial_idx}...')

for i in range(args.trial_idx, 9):
    
    sub_id = int(time.time())

    print('------------------------------------------------------------------------')
    print()
    print(f'Ready to start Trial {i} - config: {trials[i]}')
    print('Press ENTER to proceed')
    input()
    print('------------------------------------------------------------------------')

    subprocess.call(['python', './mediapipe_posedetect_gui.py', 
                        '--save-dir', subject_dir, 
                        '--trial-idx', str(i),
                        '--error-type', str(trials[i][1]), 
                        '--error-time', str(trials[i][2]), 
                        '--sub-id', str(sub_id)])


    print('------------------------------------------------------------------------')
    print()
    print(f'Ready to send transform to the robot')
    print('Recording will begin...')
    print()
    print('------------------------------------------------------------------------')

    subprocess.call(['python', './above_bed_video.py', '--save-dir', subject_dir, '--trial-idx', str(i)])


print()
print('TRIALS COMPLETE!')

# if args.start_here == 0:
#     print('---------------------------------------------')
#     print()
#     print('Make sure the subject is uncovered')
#     print('Press ENTER to capture the subject\'s pose...')
#     input()
#     print('---------------------------------------------')

#     subprocess.call(['python', './code/capture_human_pose.py', '--subject-dir', subject_dir, '--pose-dir', pose_dir, '--manikin', args.manikin])

# if args.start_here <= 1:
#     print('---------------------------------------------')
#     print()
#     print('Cover the subject with the blanket')
#     print('Press ENTER to capture blanket point cloud...')
#     input()
#     print('---------------------------------------------')


#     subprocess.call(['python', './code/capture_blanket.py', '--subject-dir', subject_dir, '--pose-dir', pose_dir, '--manikin', args.manikin])

# if args.start_here <= 2:
#     print('---------------------------------------------')
#     print()
#     print('Prep the robot for action')
#     print('Press ENTER to compute and action to uncover the target...')
#     input()
#     print('---------------------------------------------')

#     subprocess.call(['python', './code/get_action.py', '--subject-dir', subject_dir, '--pose-dir', pose_dir, '--tl-code', target_limb_code, '--manikin', args.manikin, '--approach', args.approach])


# print((time.time() - t0)/60)

# if args.start_here <= 3:
#     print('---------------------------------------------')
#     print()
#     print('Robot is ready to execute the action')
#     print('Recording will begin...')
#     print()
#     print('---------------------------------------------')

#     subprocess.call(['python', './code/above_bed_video.py', '--pose-dir', pose_dir])