#!/usr/bin/env python
"""
overwrite all tf msgs timestamps with rosbag timestamps
"""
import sys
import rosbag

print sys.argv[0]

#input_bag = "/home/oscar/ros_ws/monarch_ws/src/research_pkgs/mbot_autom_param_tuning/mbot_autom_tuning_examples/mbot_autom_tuning_amcl/ros/experimental_data/bags/29082016_10pm_without_tf_map_to_odom.bag"
#output_bag = "/home/oscar/test/output.bag"

#print 'overwriting tf msg timestamps with rosbag recorded time timestamp'

## overwrite tf msg timestamp with the time in which the message was recorded
#with rosbag.Bag(output_bag, 'w') as outbag:
    #for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        ## This also replaces tf timestamps under the assumption 
        ## that all transforms in the message share the same timestamp
        #if topic == "/tf" and msg.transforms:
            #msg.transforms[0].header.stamp = t
            ##outbag.write(topic, msg, msg.transforms[0].header.stamp)
            #outbag.write(topic, msg, t)
        #else:
            #outbag.write(topic, msg, t)

#print 'checking if tf timestamps are ok'

#b = rosbag.Bag(output_bag)

## check time difference between bag stamp and tf msg time stamp
#print ([((t-m.transforms[0].header.stamp).to_sec(), m.transforms[0].header.frame_id, m.transforms[0].child_frame_id) for (n,m,t) in b.read_messages("/tf")])[:100]
