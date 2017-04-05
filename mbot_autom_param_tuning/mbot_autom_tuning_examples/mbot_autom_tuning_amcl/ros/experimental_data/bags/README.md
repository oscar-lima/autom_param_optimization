bag descriptions
================

29082016_10pm_without_tf_map_to_odom.bag
========================================

Derived from 29082016_10pm.bag by filtering relevant topics for AMCL
and without the tf transform map to odom.

It was produced by executing the following command:

rosbag filter 29082016_10pm.bag 29082016_10pm_without_tf_map_to_odom.bag 'topic == "/cmd_vel" or topic == "/odom" or topic == "/rosout" or topic == "/rosout_agg" or topic == "/scan_front" or topic == "/scan_rear" or topic == "/vrpn_client_node/base_link_gt/pose" or topic == "/vrpn_client_node/mbot_head_gt/pose" or topic == "/tf" and m.transforms[0].child_frame_id != "odom"'


29082016_10pm_without_tf_map_to_odom_timestamps_corrected.bag
=============================================================

Derived from 29082016_10pm_without_tf_map_to_odom.bag
This bag was needed because during the experiment the rosbag tf was being broadcasted from another
PC (motion capture system PC) with a clock time not syncronized with the robot PC (21 sec ahead).

When you play the bag file and query a tf transform it say that needs interpolation into the past

Rosbag keeps track of the time at which the message was recorded. We overwrote each tf msg timestamp
with the rosbag timestamp by using the fix_rosbag_tf_timestamps.py located under this folder
