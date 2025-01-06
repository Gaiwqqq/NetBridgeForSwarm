


import rospy
from quadrotor_msgs.msg import GoalSet


def talker():
    pub = rospy.Publisher('/goalset_test', GoalSet, queue_size=10)
    rospy.init_node('goalset_test', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    data = GoalSet()
    while not rospy.is_shutdown():
        data.to_drone_ids = [0, 1]
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass