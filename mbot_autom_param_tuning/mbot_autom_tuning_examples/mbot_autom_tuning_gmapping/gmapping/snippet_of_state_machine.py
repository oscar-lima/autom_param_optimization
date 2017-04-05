 # transform map to pointcloud
            smach.StateMachine.add('TRANSFORM_MAP_TO_PCL', gbs.send_and_wait_events_combined(
                    event_in_list=[(self.occupancy_grid_to_pointcloud + '/event_in','e_trigger')],
                    event_out_list=[(self.occupancy_grid_to_pointcloud + '/event_out','e_success', True)],
                    timeout_duration=1), # wait 1 second for response from the map to pcl node
                    transitions={'success':'PUBLISH_GT_POINTCLOUD',
                                'timeout':'SHUTDOWN_SM',
                                'failure':'SHUTDOWN_SM'})
            
            # publish ground truth pointclouds
            smach.StateMachine.add('PUBLISH_GT_POINTCLOUD', gbs.send_and_wait_events_combined(
                    event_in_list=[(self.gt_point_publisher_node + '/event_in','e_trigger')],
                    event_out_list=[(self.gt_point_publisher_node + '/event_out','e_success', True)],
                    timeout_duration=1), # wait 1 second for response from the map to pcl node
                    transitions={'success':'COMPARE_POINTCLOUDS',
                                'timeout':'SHUTDOWN_SM',
                                'failure':'SHUTDOWN_SM'})
            
            # compare pointclouds
            smach.StateMachine.add('COMPARE_POINTCLOUDS', gbs.send_and_wait_events_combined(
                    event_in_list=[(self.iterative_closest_point_eval + '/event_in','e_start')],
                    event_out_list=[(self.iterative_closest_point_eval + '/event_out','e_success', True)],
                    timeout_duration=1), # wait 1 second for response from the icp node
                    transitions={'success':'SHUTDOWN_ROS_ALGORITHM',
                                'timeout':'SHUTDOWN_SM',
                                'failure':'SHUTDOWN_SM'})