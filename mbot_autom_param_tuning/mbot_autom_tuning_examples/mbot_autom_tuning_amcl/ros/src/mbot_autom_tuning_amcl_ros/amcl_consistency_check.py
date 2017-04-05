#!/usr/bin/env python

'''
Script to automate sending requests to run instances of AMCL with different RANDOM! parameters given a specific range for them
'''

import os
import random
from datetime import datetime, timedelta
import rospkg
import time

# NOTE : this comand has the same syntax as SMAC (automatic parameter tuning software)
# base_command = 'rosrun mbot_autom_param_tuning_tools smac_wrapper_node amcl no_instance 0 1.7976931348623157E308 2147483647 -1 '

# the two last numbers are used as the parameter_set and instance
base_command = 'rosrun mbot_autom_param_tuning_tools smac_wrapper_node amcl no_instance 0 1.7976931348623157E308 '

# the path for the parameter config file
amcl_ranges_config = rospkg.RosPack().get_path('mbot_autom_tuning_amcl') + '/ros/config/mbot_amcl_ranges.pcs'

def import_real_parameters(filename):
    """
    Import parameters from file and return a list of parameters on the following format:
    [[1, 2, 3],[1, 2, 3]...]
    1 = string describing the parameter
    2 = lower parameter limit
    3 = upper parameter limit
    """
    parameters = []
    for line in open(filename):
        # ignore lines that begin with #
        li=line.strip()
        if not li.startswith("#"):
            param_type = line.rstrip().split(" ", 2)[1]
            if (param_type == "real"):
                parameters.append(['-' + line.rstrip().split(" ", 1)[0],
                float(line.rstrip().split(" ", 2)[2].split('[', 1)[1].split(']')[0].split(",",2)[0]),
                float(line.rstrip().split(" ", 2)[2].split('[', 1)[1].split(']')[0].split(",",2)[1])])
            elif (param_type == "integer"):
                parameters.append(['-' + line.rstrip().split(" ", 1)[0],
                int(line.rstrip().split(" ", 2)[2].split('[', 1)[1].split(']')[0].split(",",2)[0]),
                int(line.rstrip().split(" ", 2)[2].split('[', 1)[1].split(']')[0].split(",",2)[1])])
            elif (param_type == "fixed"):
                parameters.append(['-' + line.rstrip().split(" ", 1)[0],
                line.rstrip().split(" ", 2)[2].split(' ')[0],
                line.rstrip().split(" ", 2)[2].split('[', 1)[1].split(']')[0]])
                
    return parameters

def generate_random_params(parameters):
    """
    Convert list of parameters into a single string
    It assumes that the second and third element of each package are the min and max limits
    for a random uniform distribution that will generate a value
    example :
    input :
        [['-laser_max_beams', 5, 200], ['-min_particles', 5, 980]]
    output :
        '-laser_max_beams 20 -min_particles 10'
    """
    param_string = ""
    for parameter in parameters:
        if type(parameter[1]) is int:
            param_string += parameter[0] + " " + str(int(random.uniform(parameter[1], parameter[2]))) + " "
        elif type(parameter[1]) is str:
            if parameter[1] == 'integer':
                param_string += parameter[0] + " " + str(int(parameter[2])) + " "
            elif parameter[1] == 'real':
                param_string += parameter[0] + " " + str(float(parameter[2])) + " "
            else:
                # string or bool
                #assert (parameter[2], str)
                param_string += parameter[0] + " " + parameter[2] + " "
        else:
            param_string += parameter[0] + " " + str(round(random.uniform(parameter[1], parameter[2]), 2)) + " "
    return param_string

def convert_time_human_readable(time_in_seconds):
    """
    Convert time_in_seconds seconds into days, hours, minutes
    """
    sec = timedelta(seconds=int(time_in_seconds))
    d = datetime(1, 1 , 1) + sec
    return d.day-1, d.hour, d.minute

def main():
    # Parameters
    parameters_ = import_real_parameters(amcl_ranges_config)
    # Uncomment for debug
    # print 'parameters : '
    # print parameters_
    # print '---------'
    # the amount of different random parameter sets that the user would like to try
    experiment_parameter_samples = 30 #30 TODO!: uncomment 30, 50 and os.system
    # How many times will each set be repeated
    experiment_repetitions = 50 #50
    # The duration of the recorded rosbag data, or the amount of time that you will play it
    rosbag_duration_in_seconds = 30
    time.sleep(5.0)
    # --------------------------------------------------
    print 'AMCL consistency check experiment started'
    log_filename = os.getenv("HOME") + "/.ros/amcl_consistency_log.csv"
    print 'Deleting old log data (if any) : ' + log_filename
    try:
        os.remove(log_filename)
    except OSError:
        pass
    # write header of log file
    with open(os.getenv("HOME") + "/.ros/amcl_consistency_log.csv","a") as text_file:
        text_file.write("parameter_set,instance,quality_score,parameter_string\n")
    # Estimate how much time the experiment will take to finish
    days, hours, minutes = convert_time_human_readable(rosbag_duration_in_seconds * \
        experiment_parameter_samples * experiment_repetitions)
    print '------------------\n\n'
    print "Estimated experiment total time : "
    if days == 0:
        print("%d hours, %d minutes" % (hours, minutes))
    else:
        print("%d days, %d hours, %d minutes" % (days, hours, minutes))
    print '\n\n------------------'
    time.sleep(5.0)
    # for loop to call os.system and send commands to the wrapper via terminal
    for i in range(0, experiment_parameter_samples):
        param_string_ = generate_random_params(parameters_)
        for j in range(0, experiment_repetitions):
            complete_command_to_run = base_command + str(i + 1) + " " + str(j + 1) + " " + param_string_
            print str(i + 1) + ") running command : " + complete_command_to_run
            os.system(complete_command_to_run)
        print '------'
