import math
from std_msgs.msg import String
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class MinimalSubscriber():

    def __init__(self):
        #the path to the file where the amcl_pose values will be stored,when this script is running.
        self.f2=open('./amcl.txt','a+')
    
    def callback(self,msg):
        
        roll,pitch,yaw=self.euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        self.f2.write(str(msg.pose.pose.position.x)+','+str(msg.pose.pose.position.y)+','+str(yaw))
        self.f2.write('\n')
   
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
    rospy.init_node('log_amcl_pose', anonymous=True)
    obj=MinimalSubscriber()
    sub = rospy.Subscriber( '/amcl_pose', PoseWithCovarianceStamped, obj.callback) #topic on which the estimated pose is published.
    rospy.spin()
    
if __name__ == '__main__':
    main()
