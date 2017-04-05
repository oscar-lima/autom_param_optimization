#!/usr/bin/env python

"""
Coordinates the execution of an algorithm instance which is working on rosbag data
"""

import rospy
import smach

# action lib
from smach_ros import ActionServerWrapper
from mbot_autom_param_tuning_msgs.msg import RunAlgorithmInstanceAction
from mbot_autom_param_tuning_msgs.msg import RunAlgorithmInstanceFeedback
from mbot_autom_param_tuning_msgs.msg import RunAlgorithmInstanceResult

# for send and wait for events
import mcr_states.common.basic_states as gbs

class GetActionLibParams(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['success','failure'], 
                                    input_keys=['run_algorithm_instance_goal'], 
                                    output_keys=['run_algorithm_instance_feedback', 'run_algorithm_instance_result'])

    def execute(self, userdata):
        # updating result (false until finished all actions)
        result = RunAlgorithmInstanceResult()
        result.success = True
        userdata.run_algorithm_instance_result = result
        # not using goal information for now...
        # print userdata.run_algorithm_instance_goal.rosbag_duration
        # giving feedback to the user
        feedback = RunAlgorithmInstanceFeedback()
        feedback.current_state = 'GET_ACTION_LIB_PARAMS'
        feedback.text='[run_algorithm_instance] trigger received!, getting params from actionlib'
        userdata.run_algorithm_instance_feedback = feedback
        return 'success'


class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self,  outcomes=['success'], 
                                    input_keys=['run_algorithm_instance_goal'], 
                                    output_keys=['run_algorithm_instance_feedback', 'run_algorithm_instance_result'])
        self.result = result

    def execute(self, userdata):
        result = RunAlgorithmInstanceResult()
        result.success = self.result
        userdata.run_algorithm_instance_result = result
        return 'success'


class RunAlgorithmInstance(object):
    '''
    Run an algorithm instance, based on rosbag data
    1. get goal information from action lib
    2. launch ROS algorithm
    3. play rosbag data with a robot doing "something", also stop publishing clock
    4. start the algorithm score function
    5. wait for rosbag to finish execution
    6. stop the algorithm score function, must also publish quality score, also publish clock again
    7. shudown ROS algorithm
    '''
    def __init__(self):
        # getting node parameters from param server
        # the maximum amount of time to wait for response from a node
        self.node_response_timeout = rospy.get_param('~node_response_timeout', 2.0)
        # the maximum amount of time that the algorithm takes to die when shutdown signal is received
        self.algorithm_shutdown_time = rospy.get_param('~algorithm_shutdown_timeout', 5.0)
        # the maximum expected time for the rosbag to finish execution
        self.rosbag_timeout = rospy.get_param('~rosbag_timeout', 300.0)
        
        # inform the user about the parameters that will be used
        rospy.loginfo('Node will run with the following parameters :')
        rospy.loginfo('node_response_timeout : ' + str(self.node_response_timeout))
        rospy.loginfo('algorithm_shutdown_time : ' + str(self.algorithm_shutdown_time))
        rospy.loginfo('rosbag_timeout : ' + str(self.rosbag_timeout))
        
    def start_algorithm_instance_coordinator(self):
        """
        Coordinator for the automatic parameter tuning algorith
        """
        # Create a SMACH state machine
        sm = smach.StateMachine(
             outcomes=['OVERALL_SUCCESS', 'OVERALL_FAILURE', 'OVERALL_PREEMPTED'],
             input_keys = ['run_algorithm_instance_goal'],
             output_keys = ['run_algorithm_instance_feedback', 'run_algorithm_instance_result'])
        # Open the container
        with sm:
            # 1. get goal information from action lib
            smach.StateMachine.add('GET_ACTION_LIB_PARAMS', GetActionLibParams(),
                    transitions={'success':'LAUNCH_ROS_ALGORITHM',
                                 'failure':'SET_ACTION_LIB_FAILURE'})

            # 2. launch ROS algorithm
            smach.StateMachine.add('LAUNCH_ROS_ALGORITHM', gbs.send_and_wait_events_combined(
                    event_in_list=[('node_launcher/event_in','e_start')],
                    event_out_list=[('node_launcher/event_out','e_started', True)],
                    timeout_duration=self.node_response_timeout),
                    transitions={'success':'PLAY_ROSBAG',
                                'timeout':'SET_ACTION_LIB_FAILURE',
                                'failure':'SET_ACTION_LIB_FAILURE'})

            # 3. play rosbag data with a robot doing "something", also stop publishing clock
            smach.StateMachine.add('PLAY_ROSBAG', gbs.send_and_wait_events_combined(
                    event_in_list=[('rosbag_launcher/event_in','e_start')],
                    event_out_list=[('rosbag_launcher/event_out','e_started', True)],
                    timeout_duration=self.node_response_timeout),
                    transitions={'success':'START_SCORING',
                                 'timeout':'SET_ACTION_LIB_FAILURE',
                                 'failure':'SET_ACTION_LIB_FAILURE'})
            
            # 4. start the algorithm score function
            smach.StateMachine.add('START_SCORING', gbs.send_and_wait_events_combined(
                    event_in_list=[('scoring_function/event_in','e_start')],
                    event_out_list=[('scoring_function/event_out','e_started', True)],
                    timeout_duration=self.node_response_timeout),
                    transitions={'success':'WAIT_FOR_ROSBAG_TO_FINISH_EXECUTION',
                                 'timeout':'SET_ACTION_LIB_FAILURE',
                                 'failure':'SET_ACTION_LIB_FAILURE'})

            # 5. wait for rosbag to finish execution
            smach.StateMachine.add('WAIT_FOR_ROSBAG_TO_FINISH_EXECUTION', gbs.wait_for_events(
                    [('rosbag_launcher/status','e_done', True)], timeout_duration=self.rosbag_timeout),
                      transitions={'success':'STOP_SCORING_AND_RECORD_CPU_TIME',
                                   'timeout':'SET_ACTION_LIB_FAILURE',
                                   'failure':'SET_ACTION_LIB_FAILURE'})
            
            # 6. stop the algorithm score function, must also publish quality score, also publish clock again
            smach.StateMachine.add('STOP_SCORING_AND_RECORD_CPU_TIME', gbs.send_and_wait_events_combined(
                    event_in_list=[('scoring_function/event_in','e_stop'),
                                   ('cpu_time_reader/event_in','e_start')],
                    event_out_list=[('scoring_function/event_out','e_stopped', True),
                                    ('cpu_time_reader/event_out','e_started', True)],
                    timeout_duration=self.node_response_timeout),
                    transitions={'success':'SHUTDOWN_ROS_ALGORITHM',
                                'timeout':'SET_ACTION_LIB_FAILURE',
                                'failure':'SET_ACTION_LIB_FAILURE'})
            
            # 7. shutdown ROS algorithm
            smach.StateMachine.add('SHUTDOWN_ROS_ALGORITHM', gbs.send_and_wait_events_combined(
                    event_in_list=[('node_launcher/event_in','e_stop')],
                    event_out_list=[('node_launcher/event_out','e_stopped', True)],
                    timeout_duration=self.algorithm_shutdown_time),
                    transitions={'success':'SET_ACTION_LIB_SUCCESS',
                                'timeout':'SET_ACTION_LIB_FAILURE',
                                'failure':'SET_ACTION_LIB_FAILURE'})
        
            # state machine overall success, report status trough actionlib
            smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True),
                    transitions={'success':'OVERALL_SUCCESS'})

            # state machine overall failed, report status trough actionlib
            smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False), 
                                transitions={'success':'OVERALL_FAILURE'})
        
        # Construct action server wrapper
        asw = ActionServerWrapper(
            server_name = 'run_algorithm_instance_server', 
            action_spec = RunAlgorithmInstanceAction, 
            wrapped_container = sm,
            succeeded_outcomes = ['OVERALL_SUCCESS'],
            aborted_outcomes   = ['OVERALL_FAILURE'],
            preempted_outcomes = ['OVERALL_PREEMPTED'],
            goal_key     = 'run_algorithm_instance_goal',
            feedback_key = 'run_algorithm_instance_feedback',
            result_key   = 'run_algorithm_instance_result')
        # Run the server in a background thread
        asw.run_server()
        rospy.spin()

    
def main():
    rospy.init_node('algorithm_instance_coordinator', anonymous=False)
    algorithm_instance_coordinator = RunAlgorithmInstance()
    algorithm_instance_coordinator.start_algorithm_instance_coordinator()
