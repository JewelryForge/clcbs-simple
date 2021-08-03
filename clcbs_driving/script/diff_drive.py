#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class DiffDrive:
    def __init__(self, width, radius):
        self.width, self.radius = width, radius
        self.cmd_sub = rospy.Subscriber('cmd_vel', Twist, self.callback)
        self.left_pub = rospy.Publisher('left_wheel_controller/command', Float64, queue_size=1)
        self.right_pub = rospy.Publisher('right_wheel_controller/command', Float64, queue_size=1)

    def callback(self, t):
        vx, vw = t.linear.x, t.angular.z
        vl = Float64(data=(vx - vw * self.width / 2) / self.radius)
        vr = Float64(data=(vx + vw * self.width / 2) / self.radius)
        self.left_pub.publish(vl)
        self.right_pub.publish(vr)


if __name__ == '__main__':
    args = {}
    for arg in sys.argv[1:]:
        arg_name, arg_value = arg.split(':=')
        args[arg_name] = arg_value
    assert "__name" in args
    rospy.init_node(args['__name'])
    DiffDrive(float(args.get('width', 0.718)), float(args.get('radius', 0.164)))
    rospy.spin()
