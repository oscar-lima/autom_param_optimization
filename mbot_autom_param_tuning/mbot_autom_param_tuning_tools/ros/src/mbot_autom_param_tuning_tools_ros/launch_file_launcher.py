#!/usr/bin/env python

import rospy
import roslaunch
from std_msgs.msg import String
import time

class AlgorithmLauncher(object):
    """
    Listens to event_in topic and launches or kills a launch file from code
    """
    def __init__(self):
        rospy.loginfo("Starting algorithm launcher node")
        # class variables
        self.event_in = None
        # Node cycle rate (in hz)
        self.loop_rate = rospy.get_param('~loop_rate', 30.0)
        # Publishers
        self.event_out_pub = rospy.Publisher("~event_out", String, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1)
        # Subscribers
        rospy.Subscriber("~event_in", String, self.eventInCallback)
        # get parameters
        self.launch_file_path = rospy.get_param('~launch_file_path', "/home/user/example.launch")
        # inform user about which parameters will be used
        rospy.loginfo("The following parameters will be used : ")
        rospy.loginfo("launch_file_path : " + self.launch_file_path)
        # initialize launch object
        self.launch = None
        self.running = False
        # give some time for publisher and subscriber to register in network
        time.sleep(1.)

    def eventInCallback(self, msg):
        """
        Callback for event_in msg
        """
        rospy.loginfo("event_in msg received")
        self.event_in = msg.data
        
    def main_loop(self):
        if self.running == True:
            if self.launch.pm.is_shutdown:
                rospy.loginfo("Launch file has finished execution")
                self.status_pub.publish(String('e_done'))
                self.running = False
        if self.event_in == None:
            return
        if self.event_in == 'e_start':
            # start launch file
            rospy.loginfo("Launching algorithm : " + self.launch_file_path)
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(uuid, [self.launch_file_path])
            self.launch.start()
            self.running = True
            self.event_out_pub.publish(String('e_started'))
        elif self.event_in == 'e_stop':
            # kill algorithm
            rospy.logwarn("Killing algorithm instance")
            self.launch.shutdown()
            self.event_out_pub.publish(String('e_stopped'))
        else:
            rospy.logerr('Received unsupported event message, available options are : e_start, e_stop')
            self.event_out_pub.publish(String('e_failure'))
        # reset event in flag
        self.event_in = None

    def start_algorithm_launcher(self):
        """
        Node main rutine
        """
        while not rospy.is_shutdown():
            self.main_loop()
            # sleep to control the node frequency
            time.sleep(1./self.loop_rate)


def main():
    rospy.init_node('algorithm_launcher', anonymous=False)
    algorithm_launcher = AlgorithmLauncher()
    algorithm_launcher.start_algorithm_launcher()
