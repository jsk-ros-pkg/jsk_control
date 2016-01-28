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
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <pcl/common/angles.h>
#include <boost/format.hpp>

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
    pub_text_ = nh.advertise<jsk_rviz_plugins::OverlayText>(
      "text", 1, true);
    pub_close_list_ = nh.advertise<sensor_msgs::PointCloud2>(
      "close_list", 1, true);
    pub_open_list_ = nh.advertise<sensor_msgs::PointCloud2>(
      "open_list", 1, true);
    srv_project_footprint_ = nh.advertiseService(
      "project_footprint", &FootstepPlanner::projectFootPrintService, this);
    srv_project_footprint_with_local_search_ = nh.advertiseService(
      "project_footprint_with_local_search", &FootstepPlanner::projectFootPrintWithLocalSearchService, this);
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
    JSK_ROS_DEBUG("pointcloud model is updated");
    pointcloud_model_.reset(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(*msg, *pointcloud_model_);
    if (graph_ && use_pointcloud_model_) {
      graph_->setPointCloudModel(pointcloud_model_);
    }
  }

  bool FootstepPlanner::projectFootPrint(
    const Eigen::Affine3f& center_pose,
    const Eigen::Affine3f& left_pose_trans,
    const Eigen::Affine3f& right_pose_trans,
    geometry_msgs::Pose& pose)
  {
    const Eigen::Vector3f resolution(resolution_x_,
                                     resolution_y_,
                                     resolution_theta_);
    const Eigen::Vector3f footstep_size(footstep_size_x_,
                                        footstep_size_y_,
                                        0.000001);
    Eigen::Affine3f left_pose = center_pose * left_pose_trans;
    Eigen::Affine3f right_pose = center_pose * right_pose_trans;
    FootstepState::Ptr left_input(new FootstepState(
                                    jsk_footstep_msgs::Footstep::LEFT,
                                    left_pose,
                                    footstep_size,
                                    resolution));
    FootstepState::Ptr right_input(new FootstepState(
                                    jsk_footstep_msgs::Footstep::RIGHT,
                                    right_pose,
                                    footstep_size,
                                    resolution));
    FootstepState::Ptr projected_left = graph_->projectFootstep(left_input);
    FootstepState::Ptr projected_right = graph_->projectFootstep(right_input);
    if (!projected_left || !projected_right) {
      return false;
    }
    Eigen::Affine3f projected_left_pose = projected_left->getPose();
    Eigen::Affine3f projected_right_pose = projected_right->getPose();
    Eigen::Quaternionf rot = Eigen::Quaternionf(projected_left_pose.rotation()).slerp(
      0.5, Eigen::Quaternionf(projected_right_pose.rotation()));
    Eigen::Vector3f pos = (Eigen::Vector3f(projected_right_pose.translation()) +
                           Eigen::Vector3f(projected_left_pose.translation())) / 2.0;
    Eigen::Affine3f mid = Eigen::Translation3f(pos) * rot;
    tf::poseEigenToMsg(mid, pose);
    return true;
  }
  
  bool FootstepPlanner::projectFootPrintWithLocalSearchService(
    jsk_interactive_marker::SnapFootPrint::Request& req,
    jsk_interactive_marker::SnapFootPrint::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!graph_ ) {
      return false;
    }
    if (use_pointcloud_model_ && !pointcloud_model_) {
      JSK_ROS_ERROR("No pointcloud model is yet available");
      publishText(pub_text_,
                  "No pointcloud model is yet available",
                  ERROR);
      return false;
    }
    Eigen::Affine3f center_pose, left_pose_trans, right_pose_trans;
    std::vector<Eigen::Affine3f> center_poses;
    tf::poseMsgToEigen(req.lleg_pose, left_pose_trans);
    tf::poseMsgToEigen(req.rleg_pose, right_pose_trans);
    tf::poseMsgToEigen(req.input_pose.pose, center_pose);
    const double dx = 0.05;
    const double dy = 0.05;
    const double dtheta = pcl::deg2rad(5.0);
    for (int xi = 0; xi < 3; xi++) {
      for (int yi = 0; yi < 3; yi++) {
        for (int thetai = 0; thetai < 3; thetai++) {
          Eigen::Affine3f transppp = affineFromXYYaw(xi * dx, yi * dy, thetai * dtheta);
          Eigen::Affine3f transppm = affineFromXYYaw(xi * dx, yi * dy, - thetai * dtheta);
          Eigen::Affine3f transpmp = affineFromXYYaw(xi * dx, - yi * dy, thetai * dtheta);
          Eigen::Affine3f transpmm = affineFromXYYaw(xi * dx, - yi * dy, -thetai * dtheta);
          Eigen::Affine3f transmpp = affineFromXYYaw(- xi * dx, yi * dy, thetai * dtheta);
          Eigen::Affine3f transmpm = affineFromXYYaw(- xi * dx, yi * dy, - thetai * dtheta);
          Eigen::Affine3f transmmp = affineFromXYYaw(- xi * dx, - yi * dy, thetai * dtheta);
          Eigen::Affine3f transmmm = affineFromXYYaw(- xi * dx, - yi * dy, - thetai * dtheta);
          center_poses.push_back(center_pose * transppp);
          center_poses.push_back(center_pose * transppm);
          center_poses.push_back(center_pose * transpmp);
          center_poses.push_back(center_pose * transpmm);
          center_poses.push_back(center_pose * transmpp);
          center_poses.push_back(center_pose * transmpm);
          center_poses.push_back(center_pose * transmmp);
          center_poses.push_back(center_pose * transmmm);
        }
      }
    }
    for (size_t i = 0; i < center_poses.size(); i++) {
      if (projectFootPrint(center_poses[i], left_pose_trans, right_pose_trans,
                           res.snapped_pose.pose)) {
        res.success = true;
        res.snapped_pose.header = req.input_pose.header;
        return true;
      }
    }
    JSK_ROS_ERROR("Failed to project footprint");
    publishText(pub_text_,
                "Failed to project goal",
                ERROR);
    return false;
  }

  bool FootstepPlanner::projectFootPrintService(
    jsk_interactive_marker::SnapFootPrint::Request& req,
    jsk_interactive_marker::SnapFootPrint::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!graph_) {
      return false;
    }
    if (!pointcloud_model_) {
      JSK_ROS_ERROR("No pointcloud model is yet available");
      publishText(pub_text_,
                  "No pointcloud model is yet available",
                  ERROR);
      return false;
    }
    Eigen::Affine3f center_pose, left_pose_trans, right_pose_trans;
    tf::poseMsgToEigen(req.lleg_pose, left_pose_trans);
    tf::poseMsgToEigen(req.rleg_pose, right_pose_trans);
    tf::poseMsgToEigen(req.input_pose.pose, center_pose);
    if (projectFootPrint(center_pose, left_pose_trans, right_pose_trans,
                         res.snapped_pose.pose)) {
      res.success = true;
      res.snapped_pose.header = req.input_pose.header;
      return true;
    }
    else {
      JSK_ROS_ERROR("Failed to project footprint");
      publishText(pub_text_,
                  "Failed to project goal",
                  ERROR);
      return false;
    }
  }

  void FootstepPlanner::publishText(ros::Publisher& pub,
                                    const std::string& text,
                                    PlanningStatus status)
  {
    std_msgs::ColorRGBA ok_color;
    ok_color.r = 0.3568627450980392;
    ok_color.g = 0.7529411764705882;
    ok_color.b = 0.8705882352941177;
    ok_color.a = 1.0;
    std_msgs::ColorRGBA warn_color;
    warn_color.r = 0.9411764705882353;
    warn_color.g = 0.6784313725490196;
    warn_color.b = 0.3058823529411765;
    warn_color.a = 1.0;
    std_msgs::ColorRGBA error_color;
    error_color.r = 0.8509803921568627;
    error_color.g = 0.3254901960784314;
    error_color.b = 0.30980392156862746;
    error_color.a = 1.0;
    std_msgs::ColorRGBA color;
    if (status == OK) {
      color = ok_color;
    }
    else if (status == WARNING) {
      color = warn_color;
    }
    else if (status == ERROR) {
      color = error_color;
    }
    jsk_rviz_plugins::OverlayText msg;
    msg.text = text;
    msg.width = 1000;
    msg.height = 1000;
    msg.top = 10;
    msg.left = 10;
    msg.bg_color.a = 0.0;
    msg.fg_color = color;
    msg.text_size = 24;
    pub.publish(msg);
  }

  void FootstepPlanner::planCB(
    const jsk_footstep_msgs::PlanFootstepsGoal::ConstPtr& goal)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_header_ = goal->goal_footstep.header;
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
        publishText(pub_text_,
                    "Failed to project start",
                    ERROR);

        as_.setPreempted();
        return;
      }
    }

    ////////////////////////////////////////////////////////////////////
    // set goal state
    jsk_footstep_msgs::Footstep left_goal, right_goal;
    for (size_t i = 0; i < goal->goal_footstep.footsteps.size(); i++) {
      FootstepState::Ptr goal_state(FootstepState::fromROSMsg(
                                      goal->goal_footstep.footsteps[i],
                                      footstep_size,
                                      Eigen::Vector3f(resolution_x_,
                                                      resolution_y_,
                                                      resolution_theta_)));
      if (goal_state->getLeg() == jsk_footstep_msgs::Footstep::LEFT) {
        graph_->setLeftGoalState(goal_state);
        left_goal = goal->goal_footstep.footsteps[i];
      }
      else if (goal_state->getLeg() == jsk_footstep_msgs::Footstep::RIGHT) {
        graph_->setRightGoalState(goal_state);
        right_goal = goal->goal_footstep.footsteps[i];
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
        publishText(pub_text_,
                    "Failed to project goal",
                    ERROR);
        return;
      }
    }
    // set parameters
    if (use_transition_limit_) {
      graph_->setTransitionLimit(
        TransitionLimitXYZRPY::Ptr(new TransitionLimitXYZRPY(
                                     transition_limit_x_,
                                     transition_limit_y_,
                                     transition_limit_z_,
                                     transition_limit_roll_,
                                     transition_limit_pitch_,
                                     transition_limit_yaw_)));
    }
    else {
      graph_->setTransitionLimit(TransitionLimitXYZRPY::Ptr());
    }
    if (use_global_transition_limit_) {
      graph_->setGlobalTransitionLimit(
        TransitionLimitRP::Ptr(new TransitionLimitRP(
                                     global_transition_limit_roll_,
                                     global_transition_limit_pitch_)));

    }
    else {
      graph_->setGlobalTransitionLimit(TransitionLimitRP::Ptr());
    }
    graph_->setLocalXMovement(local_move_x_);
    graph_->setLocalYMovement(local_move_y_);
    graph_->setLocalThetaMovement(local_move_theta_);
    graph_->setLocalXMovementNum(local_move_x_num_);
    graph_->setLocalYMovementNum(local_move_y_num_);
    graph_->setLocalThetaMovementNum(local_move_theta_num_);
    graph_->setPlaneEstimationMaxIterations(plane_estimation_max_iterations_);
    graph_->setPlaneEstimationMinInliers(plane_estimation_min_inliers_);
    graph_->setPlaneEstimationOutlierThreshold(plane_estimation_outlier_threshold_);
    graph_->setSupportCheckXSampling(support_check_x_sampling_);
    graph_->setSupportCheckYSampling(support_check_y_sampling_);
    graph_->setSupportCheckVertexNeighborThreshold(support_check_vertex_neighbor_threshold_);
    // Solver setup
    FootstepAStarSolver<FootstepGraph> solver(graph_,
                                              close_list_x_num_,
                                              close_list_y_num_,
                                              close_list_theta_num_,
                                              profile_period_,
                                              cost_weight_,
                                              heuristic_weight_);
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
    graph_->clearPerceptionDuration();
    solver.setProfileFunction(boost::bind(&FootstepPlanner::profile, this, _1, _2));
    ros::WallTime start_time = ros::WallTime::now();
    std::vector<SolverNode<FootstepState, FootstepGraph>::Ptr> path = solver.solve(timeout);
    ros::WallTime end_time = ros::WallTime::now();
    double planning_duration = (end_time - start_time).toSec();
    JSK_ROS_INFO_STREAM("took " << planning_duration << " sec");
    JSK_ROS_INFO_STREAM("path: " << path.size());
    if (path.size() == 0) {
      JSK_ROS_ERROR("Failed to plan path");
      publishText(pub_text_,
                  "Failed to plan",
                  ERROR);
      as_.setPreempted();
      return;
    }
    // Convert path to FootstepArray
    jsk_footstep_msgs::FootstepArray ros_path;
    ros_path.header = goal->goal_footstep.header;
    for (size_t i = 0; i < path.size(); i++) {
      ros_path.footsteps.push_back(*path[i]->getState()->toROSMsg());
    }
    // finalize path
    if (path[path.size() - 1]->getState()->getLeg() == jsk_footstep_msgs::Footstep::LEFT) {
      ros_path.footsteps.push_back(right_goal);
      ros_path.footsteps.push_back(left_goal);
    }
    else if (path[path.size() - 1]->getState()->getLeg() == jsk_footstep_msgs::Footstep::RIGHT) {
      ros_path.footsteps.push_back(left_goal);
      ros_path.footsteps.push_back(right_goal);
    }
    result_.result = ros_path;
    as_.setSucceeded(result_);

    pcl::PointCloud<pcl::PointNormal> close_list_cloud, open_list_cloud;
    solver.openListToPointCloud(open_list_cloud);
    solver.closeListToPointCloud(close_list_cloud);
    publishPointCloud(close_list_cloud, pub_close_list_, goal->goal_footstep.header);
    publishPointCloud(open_list_cloud, pub_open_list_, goal->goal_footstep.header);
    publishText(pub_text_,
                (boost::format("Took %f sec\nPerception took %f sec\nPlanning took %f sec\n%lu path\nopen list: %lu\nclose list:%lu")
                 % planning_duration 
                 % graph_->getPerceptionDuration().toSec()
                 % (planning_duration - graph_->getPerceptionDuration().toSec())
                 % path.size()
                 % open_list_cloud.points.size()
                 % close_list_cloud.points.size()).str(),
                OK);
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
    publishText(pub_text_,
                (boost::format("open_list: %lu\nclose list:%lu")
                 % (solver.getOpenList().size()) % (solver.getCloseList().size())).str(),
                OK);
    if (rich_profiling_) {
      pcl::PointCloud<pcl::PointNormal> close_list_cloud, open_list_cloud;
      solver.openListToPointCloud(open_list_cloud);
      solver.closeListToPointCloud(close_list_cloud);
      publishPointCloud(close_list_cloud, pub_close_list_, latest_header_);
      publishPointCloud(open_list_cloud, pub_open_list_, latest_header_);
    }
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
    rich_profiling_ = config.rich_profiling;
    use_transition_limit_ = config.use_transition_limit;
    use_global_transition_limit_ = config.use_global_transition_limit;
    local_move_x_ = config.local_move_x;
    local_move_y_ = config.local_move_y;
    local_move_theta_ = config.local_move_theta;
    local_move_x_num_ = config.local_move_x_num;
    local_move_y_num_ = config.local_move_y_num;
    local_move_theta_num_ = config.local_move_theta_num;
    transition_limit_x_ = config.transition_limit_x;
    transition_limit_y_ = config.transition_limit_y;
    transition_limit_z_ = config.transition_limit_z;
    transition_limit_roll_ = config.transition_limit_roll;
    transition_limit_pitch_ = config.transition_limit_pitch;
    transition_limit_yaw_ = config.transition_limit_yaw;
    global_transition_limit_roll_ = config.global_transition_limit_roll;
    global_transition_limit_pitch_ = config.global_transition_limit_pitch;
    goal_pos_thr_ = config.goal_pos_thr;
    goal_rot_thr_ = config.goal_rot_thr;
    plane_estimation_max_iterations_ = config.plane_estimation_max_iterations;
    plane_estimation_min_inliers_ = config.plane_estimation_min_inliers;
    plane_estimation_outlier_threshold_ = config.plane_estimation_outlier_threshold;
    support_check_x_sampling_ = config.support_check_x_sampling;
    support_check_y_sampling_ = config.support_check_y_sampling;
    support_check_vertex_neighbor_threshold_ = config.support_check_vertex_neighbor_threshold;
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
    cost_weight_ = config.cost_weight;
    heuristic_weight_ = config.heuristic_weight;
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

