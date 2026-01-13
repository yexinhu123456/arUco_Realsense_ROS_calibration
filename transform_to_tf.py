#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class TransformToTFNow:
    def __init__(self):
        topic = rospy.get_param("~topic", "/aruco_single/transform")
        self.br = TransformBroadcaster()
        self.sub = rospy.Subscriber(topic, TransformStamped, self.cb, queue_size=10)
        rospy.loginfo("TF bridge running (stamp=now) from %s", topic)

    def cb(self, msg: TransformStamped):
        t = TransformStamped()
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.header.stamp = rospy.Time.now()     # <-- critical for easy_handeye
        t.transform = msg.transform
        self.br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("transform_to_tf_now")
    TransformToTFNow()
    rospy.spin()
