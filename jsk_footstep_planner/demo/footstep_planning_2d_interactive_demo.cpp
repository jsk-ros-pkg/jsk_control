// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <jsk_footstep_msgs/FootstepArray.h>
#include "jsk_footstep_planner/footstep_graph.h"
#include "jsk_footstep_planner/astar_solver.h"
#include "jsk_footstep_planner/footstep_astar_solver.h"
#include <time.h>
#include <boost/random.hpp>
#include <interactive_markers/tools.h>
#include <interactive_markers/interactive_marker_server.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <boost/format.hpp>
using namespace jsk_footstep_planner;

const Eigen::Vector3f footstep_size(0.2, 0.1, 0.000001);
const Eigen::Vector3f resolution(0.05, 0.05, 0.08);
ros::Publisher pub_goal, pub_path, pub_text;
FootstepGraph::Ptr graph;

void profile(FootstepAStarSolver<FootstepGraph>& solver, FootstepGraph::Ptr graph)
{
  ROS_INFO("open list: %lu", solver.getOpenList().size());
  ROS_INFO("close list: %lu", solver.getCloseList().size());
}

void plan(const Eigen::Affine3f& goal_center,
          FootstepGraph::Ptr graph, ros::Publisher& pub_path,
          ros::Publisher& pub_goal,
          Eigen::Vector3f footstep_size)
{
  FootstepState::Ptr left_goal(new FootstepState(jsk_footstep_msgs::Footstep::LEFT,
                                                 goal_center * Eigen::Translation3f(0, 0.1, 0),
                                                 footstep_size,
                                                 resolution));
  FootstepState::Ptr right_goal(new FootstepState(jsk_footstep_msgs::Footstep::RIGHT,
                                                  goal_center * Eigen::Translation3f(0, -0.1, 0),
                                                  footstep_size,
                                                  resolution));

  jsk_footstep_msgs::FootstepArray ros_goal;
  ros_goal.header.frame_id = "odom";
  ros_goal.header.stamp = ros::Time::now();
  ros_goal.footsteps.push_back(*left_goal->toROSMsg());
  ros_goal.footsteps.push_back(*right_goal->toROSMsg());
  pub_goal.publish(ros_goal);
    
  graph->setGoalState(left_goal, right_goal);
  //AStarSolver<FootstepGraph> solver(graph);
  FootstepAStarSolver<FootstepGraph> solver(graph, 100, 100, 100);
  //solver.setHeuristic(&footstepHeuristicStraight);
  //solver.setHeuristic(&footstepHeuristicStraightRotation);
  solver.setHeuristic(boost::bind(&footstepHeuristicStepCost, _1, _2, 1.0, 0.1));
  solver.setProfileFunction(&profile);
  ros::WallTime start_time = ros::WallTime::now();
  std::vector<SolverNode<FootstepState, FootstepGraph>::Ptr> path = solver.solve();
  ros::WallTime end_time = ros::WallTime::now();
  std::cout << "took " << (end_time - start_time).toSec() << " sec" << std::endl;
  std::cout << "path: " << path.size() << std::endl;
  jsk_footstep_msgs::FootstepArray ros_path;
  ros_path.header.frame_id = "odom";
  ros_path.header.stamp = ros::Time::now();
  for (size_t i = 0; i < path.size(); i++) {
    ros_path.footsteps.push_back(*path[i]->getState()->toROSMsg());
  }
  pub_path.publish(ros_path);

}


void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  Eigen::Affine3f new_pose;
  tf::poseMsgToEigen(feedback->pose, new_pose);
  ros::WallTime start = ros::WallTime::now();
  plan(new_pose, graph, pub_path, pub_goal, footstep_size);
  ros::WallTime end = ros::WallTime::now();
  jsk_rviz_plugins::OverlayText text;
  text.bg_color.a = 0.0;
  text.fg_color.a = 1.0;
  text.fg_color.r = 25 / 255.0;
  text.fg_color.g = 1.0;
  text.fg_color.b = 250 / 255.0;
  text.line_width = 2;
  text.top = 10;
  text.left = 10;
  text.text_size = 24;
  text.width = 600;
  text.height = 100;
  text.text = (boost::format("Planning took %f sec") % (end - start).toSec()).str();
  pub_text.publish(text);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "foootstep_planning_2d");
  ros::NodeHandle nh("~");
  ros::Publisher pub_start = nh.advertise<jsk_footstep_msgs::FootstepArray>("start", 1, true);
  pub_goal = nh.advertise<jsk_footstep_msgs::FootstepArray>("goal", 1, true);
  pub_path = nh.advertise<jsk_footstep_msgs::FootstepArray>("path", 1, true);
  pub_text = nh.advertise<jsk_rviz_plugins::OverlayText>("text", 1, true);
  boost::mt19937 rng( static_cast<unsigned long>(time(0)) );
  boost::uniform_real<> xyrange(-3.0,3.0);
  boost::variate_generator< boost::mt19937, boost::uniform_real<> > pos_rand(rng, xyrange);
  boost::uniform_real<> trange(0, 2 * M_PI);
  boost::variate_generator< boost::mt19937, boost::uniform_real<> > rot_rand(rng, trange);

  graph.reset(new FootstepGraph(resolution));
  //graph->setProgressPublisher(nh, "progress");
  // set successors
  std::vector<Eigen::Affine3f> successors;
  successors.push_back(affineFromXYYaw(0, -0.2, 0));
  successors.push_back(affineFromXYYaw(0, -0.3, 0));
  successors.push_back(affineFromXYYaw(0, -0.15, 0));
  successors.push_back(affineFromXYYaw(0.2, -0.2, 0));
  successors.push_back(affineFromXYYaw(0.25, -0.2, 0));
  successors.push_back(affineFromXYYaw(0.1, -0.2, 0));
  successors.push_back(affineFromXYYaw(-0.1, -0.2, 0));
  successors.push_back(affineFromXYYaw(0, -0.2, 0.17));
  successors.push_back(affineFromXYYaw(0, -0.3, 0.17));
  successors.push_back(affineFromXYYaw(0.2, -0.2, 0.17));
  successors.push_back(affineFromXYYaw(0.25, -0.2, 0.17));
  successors.push_back(affineFromXYYaw(0.1, -0.2, 0.17));
  successors.push_back(affineFromXYYaw(0, -0.2, -0.17));
  successors.push_back(affineFromXYYaw(0, -0.3, -0.17));
  successors.push_back(affineFromXYYaw(0.2, -0.2, -0.17));
  successors.push_back(affineFromXYYaw(0.25, -0.2, -0.17));
  successors.push_back(affineFromXYYaw(0.1, -0.2, -0.17));
  FootstepState::Ptr start(new FootstepState(jsk_footstep_msgs::Footstep::LEFT,
                                             Eigen::Affine3f::Identity(),
                                             footstep_size,
                                             resolution));
  graph->setStartState(start);
  graph->setBasicSuccessors(successors);
  jsk_footstep_msgs::FootstepArray ros_start;
  ros_start.header.frame_id = "odom";
  ros_start.header.stamp = ros::Time::now();
  ros_start.footsteps.push_back(*start->toROSMsg());
  pub_start.publish(ros_start);
  interactive_markers::InteractiveMarkerServer server("footstep_projection_demo");

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/odom";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "footstep marker";

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  // int_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  // int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  // int_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  server.insert(int_marker, &processFeedback);
  server.applyChanges();
  //plan(Eigen::Affine3f::Identity() * Eigen::Translation3f(3, 3, 0), graph, pub_path, pub_goal, footstep_size);
  ros::spin();
  return 0;
}
