import rospy
import actionlib
from joy_pose_6d import JoyPose6D
from jsk_footstep_msgs.msg import PlanFootstepsAction, PlanFootstepsGoal, Footstep, FootstepArray

class JoyFootstepPlanner(JoyPose6D):
  def __init__(self):
    JoyPose6D.__init__(self, name='JoyFootstepPlanner', publish_pose=False)
    self.support_follow_view = True
    self.frame_id = rospy.get_param('~frame_id', '/map')
    self.client = actionlib.SimpleActionClient('footstep_planner', PlanFootstepsAction)
    self.footstep_pub = rospy.Publisher('/footstep', FootstepArray)
    self.client.wait_for_server()
  def joyCB(self, status, history):
    JoyPose6D.joyCB(self, status, history)
    # check if the client is busy
    # if busy, do nothing
    # if not busy, send a new goal to the actionlib planner
    if not self.pre_pose:
      return
    if self.client.simple_state == actionlib.SimpleGoalState.DONE:
      # visualizing goal
      result = self.client.get_result()
      if result:
        self.footstep_pub.publish(result.result)
      # sending new goal
      # creating goal
      initial_footsteps = FootstepArray()
      initial_footsteps.header.frame_id = self.frame_id
      initial_footsteps.header.stamp = rospy.Time(0.0)
      l_initial = Footstep()
      l_initial.leg = Footstep.LEFT
      l_initial.pose.orientation.w = 1.0
      l_initial.pose.position.y = 0.1
      initial_footsteps.footsteps.append(l_initial)
      r_initial = Footstep()
      r_initial.leg = Footstep.RIGHT
      r_initial.pose.orientation.w = 1.0
      r_initial.pose.position.y = -0.1
      initial_footsteps.footsteps.append(r_initial)
      goal = PlanFootstepsGoal()
      goal.initial_footstep = initial_footsteps

      goal_footsteps = FootstepArray()
      goal_footsteps.header.frame_id = self.frame_id
      goal_footsteps.header.stamp = rospy.Time(0.0)
      l_goal = Footstep()
      l_goal.leg = Footstep.LEFT
      l_goal.pose = self.pre_pose.pose
      r_goal = Footstep()
      r_goal.leg = Footstep.RIGHT
      r_goal.pose = self.pre_pose.pose
      goal_footsteps.footsteps.append(l_goal)
      goal_footsteps.footsteps.append(r_goal)
      goal.goal_footstep = goal_footsteps
      self.client.send_goal(goal)
    else:
      # do nothing
      pass
