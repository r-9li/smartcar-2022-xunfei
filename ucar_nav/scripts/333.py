import rospy
from geometry_msgs.msg import Twist
rospy.init_node('moving_forward', anonymous=True)
pub_vel_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
pub_duration = 10

rate_temp = rospy.Rate(20)
for i in range(0, pub_duration):
    vel_msg = Twist()
    vel_msg.linear.x = 0.2
    pub_vel_cmd.publish(vel_msg)
    rate_temp.sleep()

rospy.spin()
