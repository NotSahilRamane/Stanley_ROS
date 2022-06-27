#!/usr/bin/env python3
import rospy
from stanley_pid import Stanley


def main():
    rospy.init_node('control_pid_stanley')
    controller = Stanley()
    controller.subscribeToTopics()
    controller.publishToTopics()
    while not rospy.is_shutdown():
        if controller.state != None:
            if controller.waypoints_list:
                controller.callController()


if __name__ == u'__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass