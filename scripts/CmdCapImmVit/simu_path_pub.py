#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':

    rospy.init_node('path_pub')

    msg = Path()

    x_vec = [ 0.0   , 10.0 , 10.0 , 15.0  , 20.0   ]
    y_vec = [ 0.0   , 30.0 ,-30.0 , 0.0   , 0.0   ]
    z_vec = [ 10.0  , 10.0 , 10.0 , 10.0  , 10.0  ]

    for i in range(len(x_vec)):
        p = PoseStamped()
        p.pose.position.x = x_vec[i]
        p.pose.position.y = y_vec[i]
        p.pose.position.z = z_vec[i]
        msg.poses.append(p)

    pub = rospy.Publisher("path", Path, queue_size=2)

    rospy.sleep(1)
    rospy.loginfo("[Path pub] Published Path : ")
    rospy.loginfo("[Path pub] {}".format(msg))
    pub.publish(msg)
