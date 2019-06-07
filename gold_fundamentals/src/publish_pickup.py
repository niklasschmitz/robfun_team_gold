#!/usr/bin/python
import os
import rospy
from time import sleep
from gold_fundamentals.msg import *
import rospkg


rospack = rospkg.RosPack()
filename = os.path.join(rospack.get_path("gold_fundamentals"), "pickup.txt")
with open(filename, 'r') as f:
    exec "positions = {}".format(f.read())

print "publishing map: {}".format(positions)

node = rospy.init_node('pickup_publisher')
pub = rospy.Publisher('pickup', Goal, queue_size=1)

goal = Goal()
for row, column in positions:
    pose = Pose()
    pose.row = row
    pose.column = column
    goal.positions.append(pose)

while not rospy.is_shutdown():
    pub.publish(goal)
    sleep(0.5)
