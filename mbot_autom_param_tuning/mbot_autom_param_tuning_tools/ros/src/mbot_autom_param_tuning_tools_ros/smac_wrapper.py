#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Float64MultiArray

# to read command line args
import sys

# to sleep with wall time
import time

# to get HOME env var, and save log under /home/user/.ros
import os

# to delete params from param server
import rosparam

# actionlib client
import actionlib
from mbot_autom_param_tuning_msgs.msg import RunAlgorithmInstanceAction, RunAlgorithmInstanceGoal, RunAlgorithmInstanceResult

class CoordinatorTrigger(object):
    '''
    Exposes a console line interface to interact with ROS algorithm.
    syntax: rosrun mbot_autom_param_tuning_tools smac_wrapper_node args
    args : -param1 value -param2 value etc...
    
    1. parse command line args and upload received parameters to parameter server
    2. publish a trigger (event_in e_start signal) to a node which will run an instance of the algorithm
    3. wait until quality score is received
    4. print information in the correct smac format (smac is the 3rd party tuning software)
    5. exit node
    '''
    def __init__(self, timeout=300.0):
        print "# Initializing smac ROS wrapper node..."
        # Create publisher to trigger the algorithm run_instance_state machine
        pub_start_algorithm_instance = rospy.Publisher('/coordinate_algorithm_instance_sm/event_in', String, queue_size=1)
        # subscribe to the score function output
        rospy.Subscriber("/smac_wrapper/algorithm_instance_quality_score", Float64, self.qualityScoreCallback)
        rospy.Subscriber("/cpu_time_reader/cpu_time_info", Float64MultiArray, self.cpuTimeCallback)
        # get the namespace in which the parameters will be set from first command line argument
        self.parameter_namespace = sys.argv[1]
        print "# Parameter namespace: " + str(self.parameter_namespace)
        # to store the arguments passed over to this wrapper for logging purposes
        self.parameter_set = None
        self.instance = None
        # to store the parameters passed to this node for logging purposes
        self.param_string = ""
        # to store the quality score msg
        self.quality_score_msg = None
        # to store the cpu time consumed by the node
        self.cpu_time_msg = None
        # delete all params under the name self.parameter_namespace
        if rospy.has_param(self.parameter_namespace):
                rosparam.delete_param(self.parameter_namespace)
        # based on command line args set algorithm parameters
        self.configure_algorithm_params()
        # give some time for subscriber and publisher to register in the network
        time.sleep(0.5)
        # publish trigger to autom param tuning coordinator (make sure is running: roslaunch mbot_automatic_param_tuning tuning_test.launch)
        print "# Running action client"
        self.preempt = False
        self.server_status = 'UNKNOWN'
        self.timeout_in_secs = timeout
        self.client = actionlib.SimpleActionClient('run_algorithm_instance_server', RunAlgorithmInstanceAction)
        self.run_client()
        if self.quality_score_msg:
            print "# Quality score was succesfully received !"
            if self.cpu_time_msg:
                print "# CPU time was succesfully received !"
            print ""
            print "Result of algorithm run: SUCCESS, 0, 0, %f, 0" % self.quality_score_msg.data
            # log result
            self.log_result()
        if not self.quality_score_msg and not rospy.is_shutdown():
            print "# ERROR: Timeout, did not received quality score in " + str(self.timeout_in_secs) + " secs"

    def wait_for_result(self, target_time, delta_time=0.1):
        '''
        waits until server is done (client.simple_state = 2)
        if the loop finishes this means that the server did timeout our request
        '''
        number_of_loops = int(target_time / delta_time)
        for i in range(0, number_of_loops):
            time.sleep(delta_time)
            if self.client.simple_state == 2:
                self.server_status = 'DONE'
                break
            if rospy.is_shutdown():
                print "# ERROR: Node execution has been cancelled"
                self.preempt = True
                break

    def run_client(self):
        print "# Waiting for server to become available..."
        self.client.wait_for_server()
        print "# Server is up!"
        goal = RunAlgorithmInstanceGoal()
        goal.rosbag_duration = self.timeout_in_secs
        print "# Sending action lib goal to run_algorithm_instance_server"
        self.client.send_goal(goal)
        print "# Waiting for quality score to be received"
        print "# Server response expected in (" + str(self.timeout_in_secs) + " secs)"
        self.wait_for_result(self.timeout_in_secs)
        if not self.preempt:
            if self.server_status == 'DONE':
                my_result = RunAlgorithmInstanceResult()
                my_result = self.client.get_result()
                print "# Server responded with the following answer, success : " + str(my_result.success)
            else:
                print "# Timeout! : Server did not responded within time, sending server request to cancel goal"
                self.client.cancel_goal()
                # give some time to the server to cancel the goal
                time.sleep(3.0)
            if self.client.simple_state != 2:
                print "# Server is still busy with a request and it ignored the cancel request sent by this client"
            else:
                print "# Server is ready to take a new request"
        else:
            print "# Action client cancelled, node received shutdown signal"

    def qualityScoreCallback(self, msg):
        self.quality_score_msg = msg

    def cpuTimeCallback(self, msg):
        self.cpu_time_msg = msg

    def isfloat(self, value):
        try:
            float(value)
            return True
        except:
            return False

    def set_param(self, prefix, param, value):
        print "# Setting param : " + prefix + param + ' ' + value
        # try to parse value to double if possible
        if self.isfloat(value):
            rospy.set_param(prefix + param, float(value))
        else:
            rospy.set_param(prefix + param, value)

    def configure_algorithm_params(self):
        ready_to_set_param = False
        print "# Parameters : "
        count = 0
        for arg in sys.argv:
            # do not log first 6 arguments
            if count > 6:
                self.param_string += str(arg) + " "
            # ignore the first 6 arguments
            if count < 7:
                if count == 5:
                    self.parameter_set = arg
                if count == 6:
                    self.instance = arg
                count = count + 1
                continue
            # if an argument in the form '-string' has been previously received, execute function
            if ready_to_set_param:
                # setting parameters in parameter server
                self.set_param(self.parameter_namespace + '/', param, arg)
                # clear flag
                ready_to_set_param = False
            if arg[0] == '-':
                # remove '-' from the arg if present
                param = arg[1:]
                # try to parse param into a number
                if not self.isfloat(param):
                    # flag to indicate that the next argument is the value of the current parameter
                    ready_to_set_param = True
        print "# -----------------"

    def log_result(self):
        '''
        file will be logged as follows:
        parameter_set, instance, quality_score, cpu_total_time, user code time, kernel time, 
        child processes code time, child processes kernel time, param_string
        '''
        data_string = ""
        # 1,1,0.1031180848,"one_parameter 3.4 another_param 2.5"
        # init cpu time to -1.0 in case of not being received in callback, then user knows that there was an error
        cpu_time = "-1.0, -1.0, -1.0, -1.0, -1.0"
        # if cpu time was received in callback then log its contents
        if self.cpu_time_msg:
            # sum all elements in array, user code time + kernel time + child processes code and kernel time
            cpu_time = str(sum(self.cpu_time_msg.data))
            # log individually the 4 time components:
            # user code time, kernel time, child processes code and child kernel time
            cpu_time += ','
            for each in self.cpu_time_msg.data:
                cpu_time += str(each)
                cpu_time += ','
        data_string = str(self.parameter_set) + "," + str(self.instance) + "," +\
            str(self.quality_score_msg.data) + "," + cpu_time + '"' + self.param_string + '"'
        add_lines_to_file(data_string)

def add_lines_to_file(data_string):
    """
    add lines to the end of a file
    """
    with open(os.getenv("HOME") + "/.ros/amcl_run_instance_log.csv","a") as text_file:
        text_file.write(data_string + "\n")

def main():
    try:
        rospy.init_node('smac_wrapper', anonymous=True)
        smac_wrapper = CoordinatorTrigger(40.0)
    except rospy.ROSInterruptException:
        print "# Could not perform RUN!"
        pass
