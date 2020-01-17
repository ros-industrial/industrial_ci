# Taken from http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('spokenwords', String, queue_size=10)
rospy.init_node('node_name')
r = rospy.Rate(10)  # 10hz
while not rospy.is_shutdown():
    pub.publish("hello world")
    r.sleep()
