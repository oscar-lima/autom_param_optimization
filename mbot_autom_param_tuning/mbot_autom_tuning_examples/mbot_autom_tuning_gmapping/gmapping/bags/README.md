29082016_10pm_without_map.bag is 29082016_10pm.bag without /map and /map_metadata topics

it was produced by executing the following commands:

rosbag filter 29082016_10pm.bag output.bag "topic != '/map'"
rosbag filter temp.bag 29082016_10pm_without_map.bag "topic != '/map_metadata'"
rm temp.bag

--------------

29082016_10pm_without_tf_map_to_odom.bag is 29082016_10pm.bag with only relevant topic for amcl and without the tf transform map to odom

rosbag filter 29082016_10pm.bag 29082016_10pm_without_tf_map_to_odom.bag 'topic == "/cmd_vel" or topic == "/odom" or topic == "/rosout" or topic == "/rosout_agg" or topic == "/scan_front" or topic == "/scan_rear" or topic == "/vrpn_client_node/base_link_gt/pose" or topic == "/vrpn_client_node/mbot_head_gt/pose" or topic == "/tf" and m.transforms[0].child_frame_id != "odom"'
