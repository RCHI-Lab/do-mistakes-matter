import numpy as np
import random
import configparser
import os.path as osp

def generate_trials():
    #error type: 0=>too far out, 1=>too far in, 2=>stop moving
    #error time: 0=>early, 1=>middle, 2=>end

    #need to add code to make it so that you don't get a batch with three error trials
    trials = [(i,-1,0) for i in range(0,9)]

    if (random.sample(range(0,2),1)==0):
        firstbatch = 1
    else:
        firstbatch = 2

    firstbatchtrials = (random.sample(range(3,6),firstbatch))
    secondbatchtrials = (random.sample(range(6,9),3-firstbatch))
    
    errortrials = firstbatchtrials+secondbatchtrials

    for i in range(0,3):
        trial = errortrials[i]
        type = i
        [time] = random.sample(range(8,13),1)
        trials[trial]=(trial,i,time)

    return trials

def save_trials(trials, subject_id, save_dir):
    
    config = configparser.ConfigParser()
    config['DEFAULT'] = {}
    config['SUBJECT DATA'] = {'subject_id': subject_id}
    
    for trial in trials:
        config[f'Trial {trial[0]}'] = {
            'error_type': trial[1],
            'error_time': trial[2]
        }
    
    with open(osp.join(save_dir, 'trial_config.ini'), 'w') as configfile:
        config.write(configfile)

def read_trials(config_file):

    config = configparser.ConfigParser()
    config.read(config_file)

    trials = []
    for i in range(9):
        error_type = config[f'Trial {i}'].getint('error_type')
        error_time = config[f'Trial {i}'].getint('error_time')
        trials.append((i, error_type, error_time))
    
    return trials



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

