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
    srv_collision_bounding_box_info_ = nh.advertiseService(
      "collision_bounding_box_info", &FootstepPlanner::collisionBoundingBoxInfoService, this);
    srv_project_footstep_ = nh.advertiseService(
      "project_footstep", &FootstepPlanner::projectFootstepService, this);
    std::vector<double> lleg_footstep_offset, rleg_footstep_offset;
    if (jsk_topic_tools::readVectorParameter(nh, "lleg_footstep_offset", lleg_footstep_offset)) {
      inv_lleg_footstep_offset_ = Eigen::Vector3f(- lleg_footstep_offset[0],
                                                  - lleg_footstep_offset[1],
                                                  - lleg_footstep_offset[2]);
    } else {
      inv_lleg_footstep_offset_ = Eigen::Vector3f(0, 0, 0);
    }
    if (jsk_topic_tools::readVectorParameter(nh, "rleg_footstep_offset", rleg_footstep_offset)) {
      inv_rleg_footstep_offset_ = Eigen::Vector3f(- rleg_footstep_offset[0],
                                                  - rleg_footstep_offset[1],
                                                  - rleg_footstep_offset[2]);
    } else {
      inv_rleg_footstep_offset_ = Eigen::Vector3f(0, 0, 0);
    }
    {
      boost::mutex::scoped_lock lock(mutex_);
      if (!readSuccessors(nh)) {
        return;
      }

      ROS_INFO("building graph");
      buildGraph();
      ROS_INFO("build graph done");
    }
    sub_pointcloud_model_ = nh.subscribe("pointcloud_model", 1, &FootstepPlanner::pointcloudCallback, this);
    sub_obstacle_model_ = nh.subscribe("obstacle_model", 1, &FootstepPlanner::obstacleCallback, this);
    std::vector<double> collision_bbox_size, collision_bbox_offset;
    if (jsk_topic_tools::readVectorParameter(nh, "collision_bbox_size", collision_bbox_size)) {
      collision_bbox_size_[0] = collision_bbox_size[0];
      collision_bbox_size_[1] = collision_bbox_size[1];
      collision_bbox_size_[2] = collision_bbox_size[2];
    }
    if (jsk_topic_tools::readVectorParameter(nh, "collision_bbox_offset", collision_bbox_offset)) {
      collision_bbox_offset_ = Eigen::Affine3f::Identity() * Eigen::Translation3f(collision_bbox_offset[0],
                                                                                  collision_bbox_offset[1],
                                                                                  collision_bbox_offset[2]);
    }
    as_.start();
  }

  void FootstepPlanner::obstacleCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    obstacle_model_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *obstacle_model_);
    obstacle_model_frame_id_ = msg->header.frame_id;
    if (graph_ && use_obstacle_model_) {
      graph_->setObstacleModel(obstacle_model_);
    }
  }
  
  void FootstepPlanner::pointcloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    ROS_DEBUG("pointcloud model is updated");
    pointcloud_model_.reset(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(*msg, *pointcloud_model_);
    pointcloud_model_frame_id_ = msg->header.frame_id;
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
      ROS_ERROR("No pointcloud model is yet available");
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
    ROS_ERROR("Failed to project footprint");
    publishText(pub_text_,
                "Failed to project goal",
                ERROR);
    return false;
  }

  bool FootstepPlanner::collisionBoundingBoxInfoService(
      jsk_footstep_planner::CollisionBoundingBoxInfo::Request& req,
      jsk_footstep_planner::CollisionBoundingBoxInfo::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    res.box_dimensions.x = collision_bbox_size_[0];
    res.box_dimensions.y = collision_bbox_size_[1];
    res.box_dimensions.z = collision_bbox_size_[2];
    tf::poseEigenToMsg(collision_bbox_offset_, res.box_offset);
    return true;
  }

  bool FootstepPlanner::projectFootstepService(
    jsk_footstep_planner::ProjectFootstep::Request& req,
    jsk_footstep_planner::ProjectFootstep::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!graph_) {
      return false;
    }
    if (!pointcloud_model_) {
      ROS_ERROR("No pointcloud model is yet available");
      //publishText(pub_text_,
      //"No pointcloud model is yet available",
      //ERROR);
      return false;
    }

    const Eigen::Vector3f resolution(resolution_x_,
                                     resolution_y_,
                                     resolution_theta_);
    const Eigen::Vector3f footstep_size(footstep_size_x_,
                                        footstep_size_y_,
                                        0.000001);

    for (std::vector<jsk_footstep_msgs::Footstep>::iterator it = req.input.footsteps.begin();
         it != req.input.footsteps.end(); it++) {
      if (it->offset.x == 0.0 &&
          it->offset.y == 0.0 &&
          it->offset.z == 0.0 ) {
        if (it->leg == jsk_footstep_msgs::Footstep::LEFT) {
          it->offset.x = - inv_lleg_footstep_offset_[0];
          it->offset.y = - inv_lleg_footstep_offset_[1];
          it->offset.z = - inv_lleg_footstep_offset_[2];
        } else {
          it->offset.x = - inv_rleg_footstep_offset_[0];
          it->offset.y = - inv_rleg_footstep_offset_[1];
          it->offset.z = - inv_rleg_footstep_offset_[2];
        }
      }
      if(it->dimensions.x == 0 &&
         it->dimensions.y == 0 &&
         it->dimensions.z == 0 ) {
        it->dimensions.x = footstep_size_x_;
        it->dimensions.y = footstep_size_y_;
        it->dimensions.z = 0.000001;
      }
      FootstepState::Ptr step = FootstepState::fromROSMsg(*it, footstep_size, resolution);
      FootstepState::Ptr projected = graph_->projectFootstep(step);
      if(!!projected) {
        res.success.push_back(true);
        jsk_footstep_msgs::Footstep::Ptr p;
        if (it->leg == jsk_footstep_msgs::Footstep::LEFT) {
          p = projected->toROSMsg(inv_lleg_footstep_offset_);
        } else if (it->leg == jsk_footstep_msgs::Footstep::RIGHT) {
          p = projected->toROSMsg(inv_rleg_footstep_offset_);
        } else {
          p = projected->toROSMsg();
        }
        res.result.footsteps.push_back(*p);
      } else {
        res.success.push_back(false);
        res.result.footsteps.push_back(*it); // return the same step as in input
      }
    }
    res.result.header = req.input.header;

    return true;
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
      ROS_ERROR("No pointcloud model is yet available");
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
      ROS_ERROR("Failed to project footprint");
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
    ROS_INFO("planCB");
    // check message sanity
    if (goal->initial_footstep.footsteps.size() == 0) {
      ROS_ERROR("no initial footstep is specified");
      as_.setPreempted();
      return;
    }
    if (goal->goal_footstep.footsteps.size() != 2) {
      ROS_ERROR("Need to specify 2 goal footsteps");
      as_.setPreempted();
      return;
    }
    if (use_pointcloud_model_ && !pointcloud_model_) {
      ROS_ERROR("No pointcloud model is yet available");
      as_.setPreempted();
      return;
    }
    // check frame_id sanity
    std::string goal_frame_id = goal->initial_footstep.header.frame_id;
    if (use_pointcloud_model_) {
      // check perception cloud header
      if (goal_frame_id != pointcloud_model_frame_id_) {
        ROS_ERROR("frame_id of goal and pointcloud do not match. goal: %s, pointcloud: %s.",
                      goal_frame_id.c_str(), pointcloud_model_frame_id_.c_str());
        as_.setPreempted();
        return;
      }
    }
    if (use_obstacle_model_) {
      // check perception cloud header
      if (goal_frame_id != obstacle_model_frame_id_) {
        ROS_ERROR("frame_id of goal and obstacle pointcloud do not match. goal: %s, obstacle: %s.",
                      goal_frame_id.c_str(), obstacle_model_frame_id_.c_str());
        as_.setPreempted();
        return;
      }
    }
    Eigen::Vector3f footstep_size(footstep_size_x_, footstep_size_y_, 0.000001);
    Eigen::Vector3f search_resolution(resolution_x_, resolution_y_, resolution_theta_);
    // check goal is whether collision free
    // conevrt goal footstep into FootstepState::Ptr instance.
    if (goal->goal_footstep.footsteps.size() != 2) {
      ROS_ERROR("goal footstep should be a pair of footsteps");
      as_.setPreempted();
      return;
    }
    std::vector<jsk_footstep_msgs::Footstep > goal_ros;
    goal_ros.push_back(goal->goal_footstep.footsteps[0]);
    goal_ros.push_back(goal->goal_footstep.footsteps[1]);
    for (int i = 0; i < 2; i++) {
      if (goal_ros[i].offset.x == 0.0 &&
          goal_ros[i].offset.y == 0.0 &&
          goal_ros[i].offset.z == 0.0 ) {
        if (goal_ros[i].leg == jsk_footstep_msgs::Footstep::LEFT) {
          goal_ros[i].offset.x = - inv_lleg_footstep_offset_[0];
          goal_ros[i].offset.y = - inv_lleg_footstep_offset_[1];
          goal_ros[i].offset.z = - inv_lleg_footstep_offset_[2];
        } else {
          goal_ros[i].offset.x = - inv_rleg_footstep_offset_[0];
          goal_ros[i].offset.y = - inv_rleg_footstep_offset_[1];
          goal_ros[i].offset.z = - inv_rleg_footstep_offset_[2];
        }
      }
      if(goal_ros[i].dimensions.x == 0 &&
         goal_ros[i].dimensions.y == 0 &&
         goal_ros[i].dimensions.z == 0 ) {
        goal_ros[i].dimensions.x = footstep_size_x_;
        goal_ros[i].dimensions.y = footstep_size_y_;
        goal_ros[i].dimensions.z = 0.000001;
      }
    }

    FootstepState::Ptr first_goal = FootstepState::fromROSMsg(goal_ros[0],
                                                              footstep_size,
                                                              search_resolution);
    FootstepState::Ptr second_goal = FootstepState::fromROSMsg(goal_ros[1],
                                                               footstep_size,
                                                               search_resolution);
    if (!graph_->isSuccessable(second_goal, first_goal)) {
      ROS_ERROR("goal is non-realistic");
      as_.setPreempted();
      return;
    }
    ros::WallDuration timeout;
    if(goal->timeout.toSec() == 0.0) {
      timeout = ros::WallDuration(planning_timeout_);
    } else {
      timeout = ros::WallDuration(goal->timeout.toSec());
    }


    ////////////////////////////////////////////////////////////////////
    // set start state
    // 0 is always start
    jsk_footstep_msgs::Footstep start_ros = goal->initial_footstep.footsteps[0];
    if (start_ros.offset.x == 0.0 &&
        start_ros.offset.y == 0.0 &&
        start_ros.offset.z == 0.0 ) {
      if (start_ros.leg == jsk_footstep_msgs::Footstep::LEFT) {
        start_ros.offset.x = - inv_lleg_footstep_offset_[0];
        start_ros.offset.y = - inv_lleg_footstep_offset_[1];
        start_ros.offset.z = - inv_lleg_footstep_offset_[2];
      } else {
        start_ros.offset.x = - inv_rleg_footstep_offset_[0];
        start_ros.offset.y = - inv_rleg_footstep_offset_[1];
        start_ros.offset.z = - inv_rleg_footstep_offset_[2];
      }
    }
    FootstepState::Ptr start(FootstepState::fromROSMsg(
                               start_ros,
                               footstep_size,
                               search_resolution));
    graph_->setStartState(start);
    if (project_start_state_) {
      if (!graph_->projectStart()) {
        ROS_ERROR("Failed to project start state");
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
    for (size_t i = 0; i < goal_ros.size(); i++) {
      FootstepState::Ptr goal_state(FootstepState::fromROSMsg(
                                      goal_ros[i],
                                      footstep_size,
                                      Eigen::Vector3f(resolution_x_,
                                                      resolution_y_,
                                                      resolution_theta_)));
      if (goal_state->getLeg() == jsk_footstep_msgs::Footstep::LEFT) {
        graph_->setLeftGoalState(goal_state);
        left_goal = goal_ros[i];
      }
      else if (goal_state->getLeg() == jsk_footstep_msgs::Footstep::RIGHT) {
        graph_->setRightGoalState(goal_state);
        right_goal = goal_ros[i];
      }
      else {
        ROS_ERROR("unknown goal leg");
        as_.setPreempted();
        return;
      }
    }
    if (project_goal_state_) {
      if (!graph_->projectGoal()) {
        ROS_ERROR("Failed to project goal");
        as_.setPreempted();
        publishText(pub_text_,
                    "Failed to project goal",
                    ERROR);
        return;
      }
    }
    // set parameters
    if (parameters_.use_transition_limit) {
      graph_->setTransitionLimit(
        TransitionLimitXYZRPY::Ptr(new TransitionLimitXYZRPY(
                                     parameters_.transition_limit_x,
                                     parameters_.transition_limit_y,
                                     parameters_.transition_limit_z,
                                     parameters_.transition_limit_roll,
                                     parameters_.transition_limit_pitch,
                                     parameters_.transition_limit_yaw)));
    }
    else {
      graph_->setTransitionLimit(TransitionLimitXYZRPY::Ptr());
    }
    if (use_obstacle_model_) {
      graph_->setCollisionBBoxSize(collision_bbox_size_);
      graph_->setCollisionBBoxOffset(collision_bbox_offset_);
    }
    if (parameters_.use_global_transition_limit) {
      graph_->setGlobalTransitionLimit(
        TransitionLimitRP::Ptr(new TransitionLimitRP(
                                     parameters_.global_transition_limit_roll,
                                     parameters_.global_transition_limit_pitch)));

    }
    else {
      graph_->setGlobalTransitionLimit(TransitionLimitRP::Ptr());
    }
    graph_->setParameters(parameters_);

    graph_->setSuccessorFunction(boost::bind(&FootstepGraph::successors_original, graph_, _1, _2));
    graph_->setPathCostFunction(boost::bind(&FootstepGraph::path_cost_original, graph_, _1, _2, _3));
    //ROS_INFO_STREAM(graph_->infoString());
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
      ROS_ERROR("Unknown heuristics");
      as_.setPreempted();
      return;
    }
    graph_->clearPerceptionDuration();
    solver.setProfileFunction(boost::bind(&FootstepPlanner::profile, this, _1, _2));
    ros::WallTime start_time = ros::WallTime::now();
    std::vector<SolverNode<FootstepState, FootstepGraph>::Ptr> path = solver.solve(timeout);
    ros::WallTime end_time = ros::WallTime::now();
    double planning_duration = (end_time - start_time).toSec();
    ROS_INFO_STREAM("took " << planning_duration << " sec");
    ROS_INFO_STREAM("path: " << path.size());
    if (path.size() == 0) {
      ROS_ERROR("Failed to plan path");
      publishText(pub_text_,
                  "Failed to plan",
                  ERROR);
      as_.setPreempted();
      return;
    }
    // finalize in graph
    std::vector <FootstepState::Ptr> finalizeSteps;
    if (! (graph_->finalizeSteps((path.size() >1 ? path[path.size()-2]->getState() : FootstepState::Ptr()),
                                 path[path.size()-1]->getState(),
                                 finalizeSteps))) {
      ROS_ERROR("Failed to finalize path");
      publishText(pub_text_,
                  "Failed to finalize path",
                  ERROR);
      as_.setPreempted();
      return;
    }
    // Convert path to FootstepArray
    jsk_footstep_msgs::FootstepArray ros_path;
    ros_path.header = goal->goal_footstep.header;
    for (size_t i = 0; i < path.size(); i++) {
      const FootstepState::Ptr st = path[i]->getState();
      if (st->getLeg() == jsk_footstep_msgs::Footstep::LEFT) {
        ros_path.footsteps.push_back(*(st->toROSMsg(inv_lleg_footstep_offset_)));
      } else {
        ros_path.footsteps.push_back(*(st->toROSMsg(inv_rleg_footstep_offset_)));
      }
    }
    for (size_t i = 0; i < finalizeSteps.size(); i++) {
      const FootstepState::Ptr st = finalizeSteps[i];
      if (st->getLeg() == jsk_footstep_msgs::Footstep::LEFT) {
        ros_path.footsteps.push_back(*(st->toROSMsg(inv_lleg_footstep_offset_)));
      } else {
        ros_path.footsteps.push_back(*(st->toROSMsg(inv_rleg_footstep_offset_)));
      }
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
    ROS_INFO_STREAM("use_obstacle_model: " << graph_->useObstacleModel());
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
    if (as_.isPreemptRequested()) {
      solver.cancelSolve();
      ROS_WARN("cancelled!");
    }
    // ROS_INFO("open list: %lu", solver.getOpenList().size());
    // ROS_INFO("close list: %lu", solver.getCloseList().size());
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
      ROS_FATAL("no successors are specified");
      return false;
    }
    // read default translation from right foot to left foot
    double default_x   = 0.0;
    double default_y   = 0.0;
    double default_theta = 0.0;
    if (nh.hasParam("default_lfoot_to_rfoot_offset")) {
      std::vector<double> default_offset;
      if (jsk_topic_tools::readVectorParameter(nh, "default_lfoot_to_rfoot_offset", default_offset)) {
        default_x =     default_offset[0];
        default_y =     default_offset[1];
        default_theta = default_offset[2];
      }
    }
    // read successors
    XmlRpc::XmlRpcValue successors_xml;
    nh.param("successors", successors_xml, successors_xml);
    if (successors_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_FATAL("successors should be an array");
      return false;
    }
    for (size_t i_successors = 0; i_successors < successors_xml.size(); i_successors++) {
      XmlRpc::XmlRpcValue successor_xml;
      successor_xml = successors_xml[i_successors];
      if (successor_xml.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_FATAL("element of successors should be an dictionary");
        return false;
      }
      double x = 0;
      double y = 0;
      double theta = 0;
      if (successor_xml.hasMember("x")) {
        x = jsk_topic_tools::getXMLDoubleValue(successor_xml["x"]);
        x += default_x;
      }
      if (successor_xml.hasMember("y")) {
        y = jsk_topic_tools::getXMLDoubleValue(successor_xml["y"]);
        y += default_y;
      }
      if (successor_xml.hasMember("theta")) {
        theta = jsk_topic_tools::getXMLDoubleValue(successor_xml["theta"]);
        theta += default_theta;
      }
      Eigen::Affine3f successor =
        Eigen::Translation3f(inv_lleg_footstep_offset_[0],
                             inv_lleg_footstep_offset_[1],
                             inv_lleg_footstep_offset_[2]) *
        affineFromXYYaw(x, y, theta) *
        Eigen::Translation3f(-inv_rleg_footstep_offset_[0],
                             -inv_rleg_footstep_offset_[1],
                             -inv_rleg_footstep_offset_[2]);
      successors_.push_back(successor);
    }
    ROS_INFO("%lu successors are defined", successors_.size());
    if ((default_x != 0.0) || (default_y != 0.0) || (default_theta != 0.0)) {
      ROS_INFO("default_offset: #f(%f %f %f)", default_x, default_y, default_theta);
    }
    if ((inv_lleg_footstep_offset_[0] != 0) ||
        (inv_lleg_footstep_offset_[1] != 0) ||
        (inv_lleg_footstep_offset_[2] != 0) ) {
      ROS_INFO("left_leg_offset: #f(%f %f %f)",
               - inv_lleg_footstep_offset_[0],
               - inv_lleg_footstep_offset_[1],
               - inv_lleg_footstep_offset_[2]);
    }
    if ((inv_rleg_footstep_offset_[0] != 0) ||
        (inv_rleg_footstep_offset_[1] != 0) ||
        (inv_rleg_footstep_offset_[2] != 0) ) {
      ROS_INFO("right_leg_offset: #f(%f %f %f)",
               - inv_rleg_footstep_offset_[0],
               - inv_rleg_footstep_offset_[1],
               - inv_rleg_footstep_offset_[2]);
    }
    for (size_t i = 0; i < successors_.size(); i++) {
      Eigen::Vector3f tr = successors_[i].translation();
      float roll, pitch, yaw;
      pcl::getEulerAngles(successors_[i], roll, pitch, yaw);
      ROS_INFO("successor_%2.2d: (make-coords :pos (scale 1000 #f(%f %f 0)) :rpy (list %f 0 0))", i, tr[0], tr[1], yaw);
    }
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
    planning_timeout_ = config.planning_timeout;
    rich_profiling_ = config.rich_profiling;
    parameters_.use_transition_limit = config.use_transition_limit;
    parameters_.use_global_transition_limit = config.use_global_transition_limit;
    parameters_.local_move_x = config.local_move_x;
    parameters_.local_move_y = config.local_move_y;
    parameters_.local_move_theta = config.local_move_theta;
    parameters_.local_move_x_num = config.local_move_x_num;
    parameters_.local_move_y_num = config.local_move_y_num;
    parameters_.local_move_theta_num = config.local_move_theta_num;
    parameters_.local_move_x_offset = config.local_move_x_offset;
    parameters_.local_move_y_offset = config.local_move_y_offset;
    parameters_.local_move_theta_offset = config.local_move_theta_offset;
    parameters_.transition_limit_x = config.transition_limit_x;
    parameters_.transition_limit_y = config.transition_limit_y;
    parameters_.transition_limit_z = config.transition_limit_z;
    parameters_.transition_limit_roll = config.transition_limit_roll;
    parameters_.transition_limit_pitch = config.transition_limit_pitch;
    parameters_.transition_limit_yaw = config.transition_limit_yaw;
    parameters_.global_transition_limit_roll = config.global_transition_limit_roll;
    parameters_.global_transition_limit_pitch = config.global_transition_limit_pitch;
    parameters_.goal_pos_thr = config.goal_pos_thr;
    parameters_.goal_rot_thr = config.goal_rot_thr;
    parameters_.plane_estimation_use_normal              = config.plane_estimation_use_normal;
    parameters_.plane_estimation_normal_distance_weight  = config.plane_estimation_normal_distance_weight;
    parameters_.plane_estimation_normal_opening_angle    = config.plane_estimation_normal_opening_angle;
    parameters_.plane_estimation_min_ratio_of_inliers    = config.plane_estimation_min_ratio_of_inliers;
    parameters_.plane_estimation_max_iterations = config.plane_estimation_max_iterations;
    parameters_.plane_estimation_min_inliers = config.plane_estimation_min_inliers;
    parameters_.plane_estimation_outlier_threshold = config.plane_estimation_outlier_threshold;
    parameters_.support_check_x_sampling = config.support_check_x_sampling;
    parameters_.support_check_y_sampling = config.support_check_y_sampling;
    parameters_.support_check_vertex_neighbor_threshold = config.support_check_vertex_neighbor_threshold;
    parameters_.support_padding_x = config.support_padding_x;
    parameters_.support_padding_y = config.support_padding_y;
    parameters_.skip_cropping = config.skip_cropping;
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
    if (use_obstacle_model_ != config.use_obstacle_model) {
      use_obstacle_model_ = config.use_obstacle_model;
      need_to_rebuild_graph = true;
    }
    parameters_.obstacle_resolution = config.obstacle_resolution;
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
                                   use_local_movement_,
                                   use_obstacle_model_));
    if (use_pointcloud_model_ && pointcloud_model_) {
      graph_->setPointCloudModel(pointcloud_model_);
    }
    if (use_obstacle_model_ && obstacle_model_) {
      graph_->setObstacleModel(obstacle_model_);
    }
    //graph_->setObstacleResolution(parameters_.obstacle_resolution);
    graph_->setParameters(parameters_);
    graph_->setBasicSuccessors(successors_);
  }
}

