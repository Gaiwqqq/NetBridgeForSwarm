


import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Pose


def talker():
    pub = rospy.Publisher('/odom_test', Odometry, queue_size=10)
    rospy.init_node('odom_talker', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    odom = Odometry()
    while not rospy.is_shutdown():
        h = Header()
        h.stamp = rospy.Time.now()
        odom.header = h
        pose = Pose()
        pose.orientation.x = 1.0
        pose.orientation.y = 1.0
        pose.orientation.z = 1.0
        pose.position.x = 2.0
        pose.position.y = 2.0
        pose.position.z = 2.0
        
        odom.pose.pose = pose
        pub.publish(odom)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass