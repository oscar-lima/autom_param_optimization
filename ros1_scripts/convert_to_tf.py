import roslib
import rospy
import tf
import turtlesim.msg
import tf.msg
import geometry_msgs.msg
import math
from geometry_msgs.msg import PoseStamped

class DynamicTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage,queue_size=10) #topic on which the new transform will be published

    def callback(self,msg):

        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "map" 
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "base_link_gt"

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        tfm = tf.msg.tfMessage([t])

        self.pub_tf.publish(tfm)


if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    tfb = DynamicTFBroadcaster()
    rospy.Subscriber('/vrpn_client_node/base_link_gt/pose',PoseStamped,tfb.callback) #subscribe to the ground_truth pose topic
    rospy.spin()