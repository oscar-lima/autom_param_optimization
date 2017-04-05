#!/usr/bin/env python

import rospy
import mbot_autom_param_tuning_tools.cpu_statistics as cpu_stats
import time

from std_msgs.msg import String, Float64MultiArray, MultiArrayLayout, MultiArrayDimension

class CPUTime(object):
    """
    Listens to event_in topic and gets the cpu time of a node.
    In general it can also work for a linux process which name is known.
    """
    def __init__(self):
        rospy.loginfo("Starting cpu time reader node")
        # class variables
        self.event_in = None
        # Node cycle rate (in hz)
        self.loop_rate = rospy.get_param('~loop_rate', 10.0)
        # Publishers
        self.event_out_pub = rospy.Publisher("~event_out", String, queue_size=1)
        self.cpu_time_pub = rospy.Publisher("~cpu_time_info", Float64MultiArray, queue_size=1)
        # Subscribers
        rospy.Subscriber("~event_in", String, self.eventInCallback)
        # get parameters
        self.time_unit = rospy.get_param('~time_unit', 'ticks') # ticks or secs
        self.node_name = rospy.get_param('~node_name', 'my_node_name')
        # inform user about which parameters will be used
        rospy.loginfo("The following parameters will be used : ")
        rospy.loginfo("time_unit : " + self.time_unit)
        rospy.loginfo("node_name : " + self.node_name)
        # give some time for publisher and subscriber to register in network
        time.sleep(1.)

    def eventInCallback(self, msg):
        """
        Callback for event_in msg
        """
        self.event_in = msg
        
    def main_loop(self):
        if self.event_in == None:
            return
        if self.event_in.data == 'e_start':
            # compute cpu time and publish to topic
            msg = Float64MultiArray()
            layout = MultiArrayLayout()
            layout.dim = [MultiArrayDimension(), MultiArrayDimension(), MultiArrayDimension(), MultiArrayDimension()]
            layout.dim[0].label = "utime"
            layout.dim[1].label = "stime"
            layout.dim[2].label = "cutime"
            layout.dim[3].label = "cstime"
            msg.data = cpu_stats.get_cpu_time(self.node_name, unit=self.time_unit)
            msg.layout = layout
            self.cpu_time_pub.publish(msg)
            # publish feedback in event_out topic
            self.event_out_pub.publish(String('e_started'))
        else:
            rospy.logerr('Received unsupported event message:%s, available options are : e_start'%self.event_in.data)
            self.event_out_pub.publish(String('e_failure'))
        # reset event in flag
        self.event_in = None

    def start_cpu_time_reader(self):
        """
        Node main rutine
        """
        while not rospy.is_shutdown():
            self.main_loop()
            # sleep to control the node frequency
            time.sleep(1./self.loop_rate)

def main():
    rospy.init_node('cpu_time_reader', anonymous=False)
    cpu_time_reader = CPUTime()
    cpu_time_reader.start_cpu_time_reader()
