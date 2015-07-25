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

#include "jsk_footstep_planner/footstep_planner.h"
#include <jsk_topic_tools/log_utils.h>
#include <jsk_topic_tools/rosparam_utils.h>
#include <jsk_pcl_ros/pcl_conversion_util.h>

namespace jsk_footstep_planner
{
  FootstepPlanner::FootstepPlanner(ros::NodeHandle& nh):
    as_(nh, nh.getNamespace(),
        boost::bind(&FootstepPlanner::planCB, this, _1), false)
  {
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (nh);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&FootstepPlanner::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_close_list_ = nh.advertise<sensor_msgs::PointCloud2>(
      "close_list", 1, true);
    pub_open_list_ = nh.advertise<sensor_msgs::PointCloud2>(
      "open_list", 1, true);
    {
      boost::mutex::scoped_lock lock(mutex_);
      if (!readSuccessors(nh)) {
        return;
      }

      JSK_ROS_INFO("building graph");
      buildGraph();
      JSK_ROS_INFO("build graph done");
    }
    sub_pointcloud_model_ = nh.subscribe("pointcloud_model", 1, &FootstepPlanner::pointcloudCallback, this);
    as_.start();
  }
  
  void FootstepPlanner::pointcloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    JSK_ROS_INFO("pointcloud model is updated");
    pointcloud_model_.reset(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(*msg, *pointcloud_model_);
    if (graph_ && use_pointcloud_model_) {
      graph_->setPointCloudModel(pointcloud_model_);
    }
  }
  
  void FootstepPlanner::planCB(
    const jsk_footstep_msgs::PlanFootstepsGoal::ConstPtr& goal)
  {
    boost::mutex::scoped_lock lock(mutex_);
    JSK_ROS_INFO("planCB");
    // check message sanity
    if (goal->initial_footstep.footsteps.size() == 0) {
      JSK_ROS_ERROR("no initial footstep is specified");
      as_.setPreempted();
      return;
    }
    if (goal->goal_footstep.footsteps.size() != 2) {
      JSK_ROS_ERROR("Need to specify 2 goal footsteps");
      as_.setPreempted();
      return;
    }
    if (use_pointcloud_model_ && !pointcloud_model_) {
      JSK_ROS_ERROR("No pointcloud model is yet available");
      as_.setPreempted();
      return;
    }
    //ros::WallDuration timeout(goal->timeout.expectedCycleTime().toSec());
    ros::WallDuration timeout(10.0);
    Eigen::Vector3f footstep_size(footstep_size_x_, footstep_size_y_, 0.000001);
    ////////////////////////////////////////////////////////////////////
    // set start state
    // 0 is always start
    Eigen::Affine3f start_pose;
    tf::poseMsgToEigen(goal->initial_footstep.footsteps[0].pose, start_pose);
    FootstepState::Ptr start(FootstepState::fromROSMsg(
                               goal->initial_footstep.footsteps[0],
                               footstep_size,
                               Eigen::Vector3f(resolution_x_,
                                               resolution_y_,
                                               resolution_theta_)));
    graph_->setStartState(start);
    if (project_start_state_) {
      if (!graph_->projectStart()) {
        JSK_ROS_ERROR("Failed to project start state");
        as_.setPreempted();
        return;
      }
    }

    ////////////////////////////////////////////////////////////////////
    // set goal state
    for (size_t i = 0; i < goal->goal_footstep.footsteps.size(); i++) {
      FootstepState::Ptr goal_state(FootstepState::fromROSMsg(
                                      goal->goal_footstep.footsteps[i],
                                      footstep_size,
                                      Eigen::Vector3f(resolution_x_,
                                                      resolution_y_,
                                                      resolution_theta_)));
      if (goal_state->getLeg() == jsk_footstep_msgs::Footstep::LEFT) {
        graph_->setLeftGoalState(goal_state);
      }
      else if (goal_state->getLeg() == jsk_footstep_msgs::Footstep::RIGHT) {
        graph_->setRightGoalState(goal_state);
      }
      else {
        JSK_ROS_ERROR("unknown goal leg");
        as_.setPreempted();
        return;
      }
    }
    if (project_goal_state_) {
      if (!graph_->projectGoal()) {
        JSK_ROS_ERROR("Failed to project goal");
        as_.setPreempted();
        return;
      }
    }

    // Solver setup
    FootstepAStarSolver<FootstepGraph> solver(graph_,
                                              close_list_x_num_,
                                              close_list_y_num_,
                                              close_list_theta_num_,
                                              profile_period_);
    if (heuristic_ == "step_cost") {
      solver.setHeuristic(boost::bind(&FootstepPlanner::stepCostHeuristic, this, _1, _2));
    }
    else if (heuristic_ == "zero") {
      solver.setHeuristic(boost::bind(&FootstepPlanner::zeroHeuristic, this, _1, _2));
    }
    else if (heuristic_ == "straight") {
      solver.setHeuristic(boost::bind(&FootstepPlanner::straightHeuristic, this, _1, _2));
    }
    else if (heuristic_ == "straight_rotation") {
      solver.setHeuristic(boost::bind(&FootstepPlanner::straightRotationHeuristic, this, _1, _2));
    }
    else {
      JSK_ROS_ERROR("Unknown heuristics");
      as_.setPreempted();
      return;
    }
    solver.setProfileFunction(boost::bind(&FootstepPlanner::profile, this, _1, _2));
    
    ros::WallTime start_time = ros::WallTime::now();
    std::vector<SolverNode<FootstepState, FootstepGraph>::Ptr> path = solver.solve(timeout);
    ros::WallTime end_time = ros::WallTime::now();
    
    std::cout << "took " << (end_time - start_time).toSec() << " sec" << std::endl;
    std::cout << "path: " << path.size() << std::endl;
    
    if (path.size() == 0) {
      JSK_ROS_ERROR("Failed to plan path");
      as_.setPreempted();
      return;
    }
    JSK_ROS_INFO_STREAM("planned path: " << path.size());
    JSK_ROS_INFO_STREAM("took: " << (end_time - start_time).toSec() << " sec");
    
    // Convert path to FootstepArray
    jsk_footstep_msgs::FootstepArray ros_path;
    ros_path.header = goal->goal_footstep.header;
    for (size_t i = 0; i < path.size(); i++) {
      ros_path.footsteps.push_back(*path[i]->getState()->toROSMsg());
    }
    result_.result = ros_path;
    as_.setSucceeded(result_);

    pcl::PointCloud<pcl::PointNormal> close_list_cloud, open_list_cloud;
    solver.openListToPointCloud(open_list_cloud);
    solver.closeListToPointCloud(close_list_cloud);
    publishPointCloud(close_list_cloud, pub_close_list_, goal->goal_footstep.header);
    publishPointCloud(open_list_cloud, pub_open_list_, goal->goal_footstep.header);
    
  }
  
  void FootstepPlanner::publishPointCloud(
    const pcl::PointCloud<pcl::PointNormal>& cloud,
    ros::Publisher& pub,
    const std_msgs::Header& header)
  {
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(cloud, ros_cloud);
    ros_cloud.header = header;
    pub.publish(ros_cloud);
  }

  void FootstepPlanner::profile(FootstepAStarSolver<FootstepGraph>& solver, FootstepGraph::Ptr graph)
  {
    JSK_ROS_INFO("open list: %lu", solver.getOpenList().size());
    JSK_ROS_INFO("close list: %lu", solver.getCloseList().size());
  }
  
  double FootstepPlanner::stepCostHeuristic(
    SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph)
  {
    return footstepHeuristicStepCost(node, graph, heuristic_first_rotation_weight_,
                                     heuristic_second_rotation_weight_);
  }

  double FootstepPlanner::zeroHeuristic(
    SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph)
  {
    return footstepHeuristicZero(node, graph);
  }

  double FootstepPlanner::straightHeuristic(
    SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph)
  {
    return footstepHeuristicStraight(node, graph);
  }

  double FootstepPlanner::straightRotationHeuristic(
    SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph)
  {
    return footstepHeuristicStraightRotation(node, graph);
  }
  
  /**
     format is
       successors:
         - x: 0
           y: 0
           theta: 0
         - x: 0
           y: 0
           theta: 0
         ...
   */
  bool FootstepPlanner::readSuccessors(ros::NodeHandle& nh)
  {
    successors_.clear();
    if (!nh.hasParam("successors")) {
      JSK_ROS_FATAL("no successors are specified");
      return false;
    }

    XmlRpc::XmlRpcValue successors_xml;
    nh.param("successors", successors_xml, successors_xml);
    if (successors_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      JSK_ROS_FATAL("successors should be an array");
      return false;
    }
    for (size_t i_successors = 0; i_successors < successors_xml.size(); i_successors++) {
      XmlRpc::XmlRpcValue successor_xml;
      successor_xml = successors_xml[i_successors];
      if (successor_xml.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        JSK_ROS_FATAL("element of successors should be an dictionary");
        return false;
      }
      double x = 0;
      double y = 0;
      double theta = 0;
      if (successor_xml.hasMember("x")) {
        x = jsk_topic_tools::getXMLDoubleValue(successor_xml["x"]);
      }
      if (successor_xml.hasMember("y")) {
        y = jsk_topic_tools::getXMLDoubleValue(successor_xml["y"]);
      }
      if (successor_xml.hasMember("theta")) {
        theta = jsk_topic_tools::getXMLDoubleValue(successor_xml["theta"]);
      }
      Eigen::Affine3f successor = affineFromXYYaw(x, y, theta);
      successors_.push_back(successor);
    }
    JSK_ROS_INFO("%lu successors are defined", successors_.size());
    return true;
  }

  void FootstepPlanner::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    bool need_to_rebuild_graph = false;
    if (use_pointcloud_model_ != config.use_pointcloud_model) {
      use_pointcloud_model_ = config.use_pointcloud_model;
      need_to_rebuild_graph = true;
    }
    if (use_lazy_perception_ != config.use_lazy_perception) {
      use_lazy_perception_ = config.use_lazy_perception;
      need_to_rebuild_graph = true;
    }
    if (use_local_movement_ != config.use_local_movement) {
      use_local_movement_ = config.use_local_movement;
      need_to_rebuild_graph = true;
    }
    if (resolution_x_ != config.resolution_x) {
      resolution_x_ = config.resolution_x;
      need_to_rebuild_graph = true;
    }
    if (resolution_y_ != config.resolution_y) {
      resolution_y_ = config.resolution_y;
      need_to_rebuild_graph = true;
    }
    if (resolution_theta_ != config.resolution_theta) {
      resolution_theta_ = config.resolution_theta;
      need_to_rebuild_graph = true;
    }
    footstep_size_x_ = config.footstep_size_x;
    footstep_size_y_ = config.footstep_size_y;
    project_start_state_ = config.project_start_state;
    project_goal_state_ = config.project_goal_state;
    close_list_x_num_ = config.close_list_x_num;
    close_list_y_num_ = config.close_list_y_num;
    close_list_theta_num_ = config.close_list_theta_num;
    profile_period_ = config.profile_period;
    heuristic_ = config.heuristic;
    heuristic_first_rotation_weight_ = config.heuristic_first_rotation_weight;
    heuristic_second_rotation_weight_ = config.heuristic_second_rotation_weight;
    if (need_to_rebuild_graph) {
      if (graph_) {             // In order to skip first initialization
        buildGraph();
      }
    }
  }
  
  void FootstepPlanner::buildGraph()
  {
    graph_.reset(new FootstepGraph(Eigen::Vector3f(resolution_x_,
                                                   resolution_y_,
                                                   resolution_theta_),
                                   use_pointcloud_model_,
                                   use_lazy_perception_,
                                   use_local_movement_));
    if (use_pointcloud_model_ && pointcloud_model_) {
      graph_->setPointCloudModel(pointcloud_model_);
    }
    graph_->setBasicSuccessors(successors_);
  }
}

