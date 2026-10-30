#!/usr/bin/env python3
"""Send one map-frame pose goal; leave planning and control to move_base."""

import argparse
import math
import threading

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, required=True, help='Target map x in metres')
    parser.add_argument('--y', type=float, required=True, help='Target map y in metres')
    parser.add_argument('--yaw-deg', type=float, required=True,
                        help='Absolute heading: 0 = map +x, 90 = map +y')
    parser.add_argument('--server', default='/move_base', help='Navigation action name')
    parser.add_argument('--dry-run', action='store_true', help='Print pose without sending it')
    args = parser.parse_args(rospy.myargv()[1:])
    if not all(math.isfinite(value) for value in (args.x, args.y, args.yaw_deg)):
        parser.error('Coordinates and heading must be finite numbers')
    return args


def main():
    args = arguments()
    # A planar heading is represented as a quaternion in ROS pose messages.
    yaw = math.radians(args.yaw_deg)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = args.x
    goal.target_pose.pose.position.y = args.y
    goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
    if args.dry_run:
        print(goal)
        return 0

    rospy.init_node('send_maneuver_goal', anonymous=True)
    client = actionlib.SimpleActionClient(args.server, MoveBaseAction)
    finished = threading.Event()
    connected = threading.Event()
    result = {}
    goal_sent = False

    def cancel():
        # Cancel only this client's goal, never all goals on the server.
        if goal_sent and not finished.is_set():
            rospy.logwarn('Requesting cancellation of this maneuver goal.')
            client.cancel_goal()

    rospy.on_shutdown(cancel)

    # Use wall time for the connection deadline even when /clock is paused.
    def connect():
        if client.wait_for_server():
            connected.set()

    threading.Thread(target=connect, daemon=True).start()
    rospy.loginfo('Waiting up to 10 wall-clock seconds for %s...', args.server)
    if not connected.wait(10.0):
        rospy.logerr('Navigation action unavailable. Start navigation first.')
        return 1
    if rospy.is_shutdown():
        return 1

    def done(state, unused_result):
        result['state'] = state
        result['text'] = client.get_goal_status_text()
        finished.set()

    rospy.loginfo('Sending map goal: x=%.3f, y=%.3f, heading=%.1f degrees',
                  args.x, args.y, args.yaw_deg)
    goal.target_pose.header.stamp = rospy.Time.now()
    goal_sent = True
    try:
        client.send_goal(goal, done_cb=done)
        rospy.loginfo('Goal sent. Ctrl+C cancels. Do not send another RViz goal during this test.')
        while not finished.wait(0.1):
            if rospy.is_shutdown():
                return 1
        if result['state'] == GoalStatus.SUCCEEDED:
            rospy.loginfo('Navigation reports SUCCESS (status 3). Check that the chair stays stopped.')
            return 0
        rospy.logwarn('Navigation ended with status %s: %s', result['state'], result['text'])
        return 1
    finally:
        cancel()
        if goal_sent and not finished.is_set():
            rospy.logwarn('Cancellation requested; stopped motion has not been verified.')


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except rospy.ROSInterruptException:
        raise SystemExit(1)
