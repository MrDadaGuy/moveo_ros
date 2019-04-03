#! /usr/bin/env python

import rospy

import actionlib
import actionlib.simple_action_server
from control_msgs.msg import *
import trajectory_msgs.msg._JointTrajectory

import control_msgs.msg._FollowJointTrajectoryFeedback
import control_msgs.msg._FollowJointTrajectoryResult

#import control_msgs.msg._GripperCommandAction


class RobotAction(object):
    # create messages that are used to publish feedback/result
    _feedback = FollowJointTrajectoryActionFeedback()
    _result = FollowJointTrajectoryActionResult()

    def __init__(self, name):
        rospy.loginfo("Hey i'm initializin' here")
        self._action_name = name
        self._as = actionlib.ActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction, self.on_goal, self.on_cancel, auto_start = False)    # execute_cb=self.execute_cb,
        self._as.start()

    def on_cancel(self, some_shit):
        rospy.loginfo("called on_cancel.  boo.  not cool.")
      
    def on_goal(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing callback - goal:  ' % self._action_name)

        points = goal.get_goal().trajectory.points

        for i in range(1, len(points)):
            rospy.loginfo(str(points[i].positions))


        # publish the feedback
#        self._as.publish_feedback(self._as., "feedback")
        # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        r.sleep()

          
        if success:
#            self._result.status = "hooray"
#            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
#            self._as.set_succeeded(self._result)
#            self._as.
        
if __name__ == '__main__':
    rospy.init_node('follow_joint_trajectory')
    server = RobotAction(rospy.get_name())
    rospy.spin()