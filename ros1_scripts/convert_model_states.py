#! /usr/bin/env python
import rospy
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from geometry_msgs.msg import PoseStamped

rospy.init_node('ground_truth_publisher')
rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
model = GetModelStateRequest()
model.model_name='turtlebot3_burger' #change the model if needed
ground_truth_publisher = rospy.Publisher("/vrpn_client_node/base_link_gt/pose", PoseStamped, queue_size=5) #topic on which ground_truth_pose is published.
counter=0

while not rospy.is_shutdown():
	result = get_model_srv(model)
	counter+=1
	gt_pose = PoseStamped()
	gt_pose.header.seq = counter
	gt_pose.header.stamp = rospy.Time.now()
	gt_pose.header.frame_id = "map"
	gt_pose.pose.position.x=result.pose.position.x
	gt_pose.pose.position.y=result.pose.position.y
	gt_pose.pose.position.z=result.pose.position.z
	gt_pose.pose.orientation.x=result.pose.orientation.x
	gt_pose.pose.orientation.y=result.pose.orientation.y
	gt_pose.pose.orientation.z=result.pose.orientation.z
	gt_pose.pose.orientation.w=result.pose.orientation.w
	ground_truth_publisher.publish(gt_pose)