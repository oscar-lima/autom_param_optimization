#!/usr/bin/env python

import roslaunch
import rospy
import time
import rosparam
import yaml
from std_msgs.msg import String

class NodeLauncher(object):
    """
    Launches a node from code when received even_it msg, it can also
    terminate the node if required by publishing on event_in msg 'e_stop' msg
    """
    def __init__(self):
        rospy.loginfo("Starting algorithm launcher node")
        # class variables
        self.event_in = None
        self.process = None
        self.launch = None
        self.running = False
        # Publishers
        self.event_out_pub = rospy.Publisher("~event_out", String, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1)
        # Subscribers
        rospy.Subscriber("~event_in", String, self.eventInCallback)
        # give some time for publisher and subscriber to register in network
        time.sleep(0.5)
        # Parameters
        self.loop_rate = rospy.get_param('~loop_rate', 30.0)
        node_pkg = rospy.get_param('~node_pkg', 'my_pkg')
        node_type = rospy.get_param('~node_type', 'my_node_type')
        self.node_name = rospy.get_param('~node_name', 'my_node_name')
        self.remap_args = rospy.get_param('~remap_args', '')
        self.node_args = rospy.get_param('~node_args', '')
        self.node_params = rospy.get_param('~node_params', '/home/user/my_params.yaml') # yaml full path file
        # inform user about which parameters will be used
        rospy.loginfo("The following parameters will be used: ")
        rospy.loginfo("loop_rate : " + str(self.loop_rate))
        rospy.loginfo("node_pkg : " + node_pkg)
        rospy.loginfo("node_type : " + node_type)
        rospy.loginfo("node_name : " + self.node_name)
        rospy.loginfo("remap_args : " + str(self.remap_args))
        if self.node_args == "":
            rospy.loginfo("node_args : no node args are set")
        else: 
            rospy.loginfo("node_args : " + self.node_args)
        rospy.loginfo("node_params : " + self.node_params)
        # create node
        self.node = roslaunch.core.Node(node_pkg, node_type)
        self.node.name = self.node_name
        if self.node_args != "":
            self.node.args = self.node_args
        if self.remap_args != "":
            remap_pair_list = []
            for i in range(0, len(self.remap_args) / 2):
                remap_pair_list.append([self.remap_args[i*2], self.remap_args[(i*2)+1]])
            self.node.remap_args = remap_pair_list
        
    def eventInCallback(self, msg):
        """
        Callback for event_in msg
        """
        rospy.loginfo("event_in msg received")
        self.event_in = msg.data
        
    def main_loop(self):
        if self.running == True:
            if not self.process.is_alive():
                rospy.loginfo("Launch file has finished execution")
                self.status_pub.publish(String('e_done'))
                self.running = False
        if self.event_in == None:
            return
        if self.event_in == 'e_start':
            # upload node parameters to param server
            self.load_params_from_yaml(self.node_params)
            # start launch file
            rospy.loginfo("Launching algorithm")
            self.launch = roslaunch.scriptapi.ROSLaunch()
            self.launch.start()
            self.process = self.launch.launch(self.node)
            self.running = True
            self.event_out_pub.publish(String('e_started'))
        elif self.event_in == 'e_stop':
            # kill node
            rospy.logwarn("Killing node instance")
            self.process.stop()
            self.event_out_pub.publish(String('e_stopped'))
        else:
            rospy.logerr('Received unsupported event message, available options are : e_start, e_stop')
            self.event_out_pub.publish(String('e_failure'))
        # reset event in flag
        self.event_in = None
        
    def load_params_from_yaml(self, complete_file_path):
        f = open(complete_file_path, 'r')
        yaml_file = yaml.load(f)
        f.close()
        rosparam.upload_params(self.node_name, yaml_file)
        
    def start_node_launcher(self):
        """
        Node main rutine
        """
        while not rospy.is_shutdown():
            self.main_loop()
            # sleep to control the node frequency
            time.sleep(1./self.loop_rate)


def main():
    rospy.init_node('algorithm_launcher', anonymous=False)
    algorithm_launcher = NodeLauncher()
    algorithm_launcher.start_node_launcher()
