

import rospy
from std_msgs.msg import String

start_received = False

def callback(data):
    global start_received
    start_received = True
    rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.data)

def listener():
    pub_start = rospy.Publisher('start', String, queue_size=10)
    rospy.init_node('listener', anonymous=True)


    rospy.Subscriber('chatter', String, callback)
    rate = rospy.Rate(10) # 10hz

    while not start_received:
        pub_start.publish('start')
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    listener()
