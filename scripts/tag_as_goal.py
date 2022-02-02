#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from ar_track_alvar_msgs.msg import AlvarMarkers

class tag_as_goal():

  def __init__(self):

    self.goalReached = False
    # initiliaze
    rospy.init_node('tag_as_goal', anonymous=False)
    self.ar_pose_marker_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.marker_cb)
    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #wait for the action server to come up
    while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
      rospy.loginfo("Waiting for the move_base action server to come up")

  def is_valid_marker_id(self, marker):
      rospy.logdebug_once("Marker ID: {}".format(marker.id))
      return marker.id == 3

  def marker_cb(self, msg):
          markers = msg.markers
          if len(markers) > 0:
              marker = markers[0]
              if self.is_valid_marker_id(marker):
                  marker_pose = marker.pose.pose
                  self.moveToGoal(marker_pose.x, marker_pose.y)
          else:
              pass

  def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Quit program")
        rospy.sleep()

  def moveToGoal(self,xGoal,yGoal):
      goal = MoveBaseGoal()

      #set up the frame parameters
      goal.target_pose.header.frame_id = "map"
      goal.target_pose.header.stamp = rospy.Time.now()

      # moving towards the goal*/
      goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
      goal.target_pose.pose.orientation.x = 0.0
      goal.target_pose.pose.orientation.y = 0.0
      goal.target_pose.pose.orientation.z = 0.0
      goal.target_pose.pose.orientation.w = 1.0

      rospy.loginfo("Sending goal location ...")
      self.ac.send_goal(goal)

      self.ac.wait_for_result(rospy.Duration(60))

      if(self.ac.get_state() ==  GoalStatus.SUCCEEDED):
              rospy.loginfo("You have reached the destination")
              return True

      else:
              rospy.loginfo("The robot failed to reach the destination")
              return False

if __name__ == '__main__':
    try:

        rospy.loginfo("You have reached the destination")
        tag_as_goal()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("tag_as_goal node terminated.")
