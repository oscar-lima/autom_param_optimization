#! /usr/bin/env python
import rospy
import roslib
import actionlib
import time

from mbot_autom_param_tuning_msgs.msg import RunAlgorithmInstanceAction, RunAlgorithmInstanceGoal, RunAlgorithmInstanceResult

class RunAmclInstanceClient(object):
    '''
    actionlib client to test the run amcl instance server
    '''
    def __init__(self, timeout=300.0):
        self.preempt = False
        self.server_status = 'UNKNOWN'
        self.client = actionlib.SimpleActionClient('run_algorithm_instance_server', RunAlgorithmInstanceAction)
        self.timeout_in_secs = timeout
        
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
                self.preempt = True
                break
        
    def start_run_algorithm_instance_client(self):
        rospy.loginfo('Waiting for server : run_algorithm_instance_server')
        self.client.wait_for_server()
        rospy.loginfo('Server is up!')
        goal = RunAlgorithmInstanceGoal()
        goal.rosbag_duration = self.timeout_in_secs
        rospy.loginfo('Sending action lib goal to run_algorithm_instance_server')
        self.client.send_goal(goal)
        rospy.loginfo('Waiting for server response (' + str(self.timeout_in_secs) + ' secs)')
        self.wait_for_result(self.timeout_in_secs)
        if not self.preempt:
            if self.server_status == 'DONE':
                my_result = RunAlgorithmInstanceResult()
                my_result = self.client.get_result()
                rospy.loginfo('Server responded with the following answer, success : ' + str(my_result.success))
            else:
                rospy.logwarn('Timeout! : Server did not responded within time, sending server request to cancel goal')
                self.client.cancel_goal()
                # give some time to the server to cancel the goal
                time.sleep(3.0)
            if self.client.simple_state != 2:
                rospy.logwarn('Server is still busy with a request and it ignored the cancel request sent by this client')
            else:
                rospy.loginfo('Server is ready to take a new request')
        else:
            rospy.logwarn('Action client cancelled, node received shutdown signal')

def main():
    rospy.init_node('run_amcl_instance_client_tester', anonymous=True)
    run_algorithm_instance_client = RunAmclInstanceClient(40.0) # we are playing only 30 secs of the rosbag, but lets give some tolerance
    run_algorithm_instance_client.start_run_algorithm_instance_client()
