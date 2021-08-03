#!/usr/bin/env python
import rospy
import tf2_ros
import sys
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import TransformStamped


class GazeboLinkPose:
    def __init__(self, *argv):
        if not argv:
            raise ValueError("Need at least a topic name")
        self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
        self.topics = []
        for topic in argv:
            if topic[:2] == '__':
                continue
            topic.strip('/')
            self.topics.append(topic)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.counter = {}

    def callback(self, data):
        for topic in self.topics:
            robot, link = topic.split('/')
            try:
                idx = data.name.index(robot + "::" + link)
                p = data.pose[idx]
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "map"
                t.child_frame_id = topic
                t.transform.translation.x = p.position.x
                t.transform.translation.y = p.position.y
                t.transform.translation.z = p.position.z
                t.transform.rotation.w = p.orientation.w
                t.transform.rotation.x = p.orientation.x
                t.transform.rotation.y = p.orientation.y
                t.transform.rotation.z = p.orientation.z
                self.broadcaster.sendTransform(t)
            except ValueError:
                self.counter[topic] = self.counter.get(topic, 0) + 1
                if self.counter[topic] % 1000 == 0:
                    rospy.logwarn(topic + ' NOT FOUND')


if __name__ == '__main__':
    try:
        rospy.init_node('state_publisher')
        gp = GazeboLinkPose(*sys.argv[1:])
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
