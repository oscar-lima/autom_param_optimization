import message_filters
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
import time
import math
import rospy

class MinimalSubscriber():

    def __init__(self):
        
        self.gt_pose_sub = message_filters.Subscriber('/vrpn_client_node/base_link_gt/pose',PoseStamped)
        self.amcl_pose_sub = message_filters.Subscriber('/amcl_pose',PoseWithCovarianceStamped)
        #ApproximateTimeSynchronizer is used to synchronize the two topics.
        self.ts = message_filters.ApproximateTimeSynchronizer([self.gt_pose_sub, self.amcl_pose_sub], 3,slop=0.1)
        self.ts.registerCallback(self.callback)
        #the path to the files where the synchronized amcl_pose and gt_pose will be stored when this script is running.
        self.f2=open('./amcl_error.txt','a+') 
        self.f1=open('./odom_error.txt','a+')
        
    def callback(self, odo_p,amcl_p):

        msg=odo_p
        roll,pitch,yaw=self.euler_from_quaternion(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
        self.f2.write(str(msg.pose.position.x)+','+str(msg.pose.position.y)+','+str(yaw))
        self.f2.write('\n')
        msg=amcl_p
        roll,pitch,yaw=self.euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        self.f1.write(str(msg.pose.pose.position.x)+','+str(msg.pose.pose.position.y)+','+str(yaw))
        self.f1.write('\n')
       

    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in meters,meters and radians respectively.

def main(args=None):
    rospy.init_node('synchronization_node', anonymous=True)
    obj=MinimalSubscriber()
    rospy.spin()

if __name__ == '__main__':
    main()
