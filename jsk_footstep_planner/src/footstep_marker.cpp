// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

#include "jsk_footstep_planner/footstep_marker.h"
#include <jsk_topic_tools/log_utils.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <jsk_interactive_marker/interactive_marker_helpers.h>
#include <boost/format.hpp>
#include <pcl/common/eigen.h>
#include <angles/angles.h>
#include "jsk_footstep_planner/footstep_conversions.h"
#include <jsk_topic_tools/rosparam_utils.h>
#include <jsk_interactive_marker/SnapFootPrint.h>
#include <jsk_footstep_planner/CollisionBoundingBoxInfo.h>

namespace jsk_footstep_planner
{

  void add3Dof2DControl( visualization_msgs::InteractiveMarker &msg, bool fixed)
  {
    visualization_msgs::InteractiveMarkerControl control;

    if(fixed)
      control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    // msg.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    msg.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    msg.controls.push_back(control);
    // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    // msg.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    // msg.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    msg.controls.push_back(control);

  }

  
  PosePair::PosePair(const Eigen::Affine3f& first, const std::string& first_name,
                     const Eigen::Affine3f& second, const std::string& second_name):
    first_(first), first_name_(first_name),
    second_(second), second_name_(second_name)
  {

  }

  Eigen::Affine3f PosePair::getByName(const std::string& name)
  {
    if (first_name_ == name) {
      return first_;
    }
    else if (second_name_ == name) {
      return second_;
    }
    else {
      throw UnknownPoseName();
    }
  }

  Eigen::Affine3f PosePair::midcoords()
  {
    Eigen::Translation3f pos((Eigen::Vector3f(first_.translation()) + Eigen::Vector3f(second_.translation())) / 2.0);
    Eigen::Quaternionf rot = Eigen::Quaternionf(first_.rotation()).slerp(0.5, Eigen::Quaternionf(second_.rotation()));
    return pos * rot;
  }
  
  FootstepMarker::FootstepMarker():
    pnh_("~"), ac_planner_("footstep_planner", true), ac_exec_("footstep_controller", true),
    pub_marker_array_(pnh_, "marker_array"),
    is_2d_mode_(true), is_cube_mode_(false),
    planning_state_(NOT_STARTED)
  {
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&FootstepMarker::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    tf_client_.reset(new tf2_ros::BufferClient("tf2_buffer_server"));
    server_.reset(new interactive_markers::InteractiveMarkerServer(ros::this_node::getName()));
    pnh_.param("frame_id", odom_frame_id_, std::string("odom"));
    pnh_.param("lleg_end_coords", lleg_end_coords_, std::string("lleg_end_coords"));
    pnh_.param("rleg_end_coords", rleg_end_coords_, std::string("rleg_end_coords"));
    pnh_.param("footstep_size_x", foot_size_x_, 0.24);
    pnh_.param("footstep_size_y", foot_size_y_, 0.14);
    pnh_.param("footstep_size_z", foot_size_z_, 0.01);
    std::vector<double> lleg_footstep_offset, rleg_footstep_offset;
    if (jsk_topic_tools::readVectorParameter(pnh_, "lleg_footstep_offset", lleg_footstep_offset)) {
      lleg_footstep_offset_[0] = lleg_footstep_offset[0];
      lleg_footstep_offset_[1] = lleg_footstep_offset[1];
      lleg_footstep_offset_[2] = lleg_footstep_offset[2];
    }
    if (jsk_topic_tools::readVectorParameter(pnh_, "rleg_footstep_offset", rleg_footstep_offset)) {
      rleg_footstep_offset_[0] = rleg_footstep_offset[0];
      rleg_footstep_offset_[1] = rleg_footstep_offset[1];
      rleg_footstep_offset_[2] = rleg_footstep_offset[2];
    }
    
    // query bbox size and offset
    ros::NodeHandle nh;
    ros::ServiceClient bbox_client = nh.serviceClient<jsk_footstep_planner::CollisionBoundingBoxInfo>("footstep_planner/collision_bounding_box_info", false);
    ROS_INFO("waiting for %s", bbox_client.getService().c_str());
    bbox_client.waitForExistence();
    jsk_footstep_planner::CollisionBoundingBoxInfo bbox_info;
    if (bbox_client.call(bbox_info)) {
      collision_bbox_size_[0] = bbox_info.response.box_dimensions.x;
      collision_bbox_size_[1] = bbox_info.response.box_dimensions.y;
      collision_bbox_size_[2] = bbox_info.response.box_dimensions.z;
      tf::poseMsgToEigen(bbox_info.response.box_offset, collision_bbox_offset_);
    }

    // pose stamped command interface
    sub_pose_stamped_command_ = pnh_.subscribe("pose_stamped_command", 1, &FootstepMarker::poseStampedCommandCallback, this);

    // service servers
    srv_reset_marker_ = pnh_.advertiseService("reset_marker", &FootstepMarker::resetMarkerService, this);
    srv_execute_footstep_ = pnh_.advertiseService("execute_footstep", &FootstepMarker::executeFootstepService, this);
    srv_get_footstep_marker_pose_ = pnh_.advertiseService("get_footstep_marker_pose", &FootstepMarker::getFootstepMarkerPoseService, this);

    pub_plan_result_ = pnh_.advertise<jsk_footstep_msgs::FootstepArray>("output/plan_result", 1);
    //JSK_ROS_INFO("waiting for footstep_planner");
    //ac_planner_.waitForServer();
    //JSK_ROS_INFO("waiting for footstep_controller");
    //ac_exec_.waitForServer();
    // initialize interactive marker
    // build menu handler
    setupMenuHandler();
    resetInteractiveMarker();
    JSK_ROS_INFO("initialization done");
  }

  FootstepMarker::~FootstepMarker()
  {
    pub_marker_array_.clear();
    pub_marker_array_.publish();
    ros::Duration(1.0).sleep();
  }
  
  visualization_msgs::Marker FootstepMarker::makeFootstepMarker(Eigen::Affine3f pose)
  {
    visualization_msgs::Marker marker;
    if (is_cube_mode_) {
      marker.type = visualization_msgs::Marker::CUBE;
      marker.scale.x = foot_size_x_;
      marker.scale.y = foot_size_y_;
      marker.scale.z = foot_size_z_;
      tf::poseEigenToMsg(pose, marker.pose);
    }
    else {
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.scale.x = 0.01;
      Eigen::Vector3f local_a( foot_size_x_ / 2.0,  foot_size_y_ / 2.0, 0.0);
      Eigen::Vector3f local_b(-foot_size_x_ / 2.0,  foot_size_y_ / 2.0, 0.0);
      Eigen::Vector3f local_c(-foot_size_x_ / 2.0, -foot_size_y_ / 2.0, 0.0);
      Eigen::Vector3f local_d( foot_size_x_ / 2.0, -foot_size_y_ / 2.0, 0.0);
      Eigen::Vector3f a = pose * local_a;
      Eigen::Vector3f b = pose * local_b;
      Eigen::Vector3f c = pose * local_c;
      Eigen::Vector3f d = pose * local_d;
      geometry_msgs::Point ros_a, ros_b, ros_c, ros_d;
      ros_a.x = a[0]; ros_a.y = a[1]; ros_a.z = a[2];
      ros_b.x = b[0]; ros_b.y = b[1]; ros_b.z = b[2];
      ros_c.x = c[0]; ros_c.y = c[1]; ros_c.z = c[2];
      ros_d.x = d[0]; ros_d.y = d[1]; ros_d.z = d[2];
      marker.points.push_back(ros_a);
      marker.points.push_back(ros_b);
      marker.points.push_back(ros_c);
      marker.points.push_back(ros_d);
      marker.points.push_back(ros_a);
    }
    
    marker.color.a = 1.0;
    return marker;
  }
  
  void FootstepMarker::setupInitialMarker(PosePair::Ptr leg_poses,
                                          visualization_msgs::InteractiveMarker& int_marker)
  {
    Eigen::Affine3f midcoords = leg_poses->midcoords();
    tf::poseEigenToMsg(midcoords, int_marker.pose);
    visualization_msgs::Marker left_box_marker = makeFootstepMarker(midcoords.inverse() * leg_poses->getByName(lleg_end_coords_));
    left_box_marker.color.g = 1.0;
    visualization_msgs::Marker right_box_marker = makeFootstepMarker(midcoords.inverse() * leg_poses->getByName(rleg_end_coords_));
    right_box_marker.color.r = 1.0;
    visualization_msgs::InteractiveMarkerControl left_box_control;
    left_box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    left_box_control.always_visible = true;
    left_box_control.markers.push_back(left_box_marker);
    int_marker.controls.push_back(left_box_control);
    visualization_msgs::InteractiveMarkerControl right_box_control;
    right_box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    right_box_control.always_visible = true;
    right_box_control.markers.push_back(right_box_marker);
    int_marker.controls.push_back(right_box_control);
    int_marker.name = "initial_footstep_marker";
    int_marker.description = "Initial Footsteps";
  }
  
  void FootstepMarker::setupGoalMarker(Eigen::Affine3f pose,
                                       visualization_msgs::InteractiveMarker& int_goal_marker)
  {
    int_goal_marker.name = "movable_footstep_marker";
    int_goal_marker.description = "Goal Footsteps";
    tf::poseEigenToMsg(pose, int_goal_marker.pose);
    Eigen::Affine3f lleg_offset = pose.inverse() * lleg_goal_pose_;
    Eigen::Affine3f rleg_offset = pose.inverse() * rleg_goal_pose_;
    visualization_msgs::Marker left_box_marker = makeFootstepMarker(lleg_offset);
    left_box_marker.color.g = 1.0;
    visualization_msgs::Marker right_box_marker = makeFootstepMarker(rleg_offset);
    right_box_marker.color.r = 1.0;
    visualization_msgs::InteractiveMarkerControl left_box_control;
    left_box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    left_box_control.always_visible = true;
    left_box_control.markers.push_back(left_box_marker);
    int_goal_marker.controls.push_back(left_box_control);
    visualization_msgs::InteractiveMarkerControl right_box_control;
    right_box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    right_box_control.always_visible = true;
    right_box_control.markers.push_back(right_box_marker);
    int_goal_marker.controls.push_back(right_box_control);
  }
  
  void FootstepMarker::resetInteractiveMarker()
  {
    PosePair::Ptr leg_poses;
    if (disable_tf_) {
      leg_poses = getDefaultFootstepPair();
    }
    else {
      leg_poses = getLatestCurrentFootstepPoses();
    }
    original_foot_poses_ = leg_poses;
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = odom_frame_id_;
    
    setupInitialMarker(leg_poses, int_marker);
    server_->insert(int_marker,
                    boost::bind(&FootstepMarker::processFeedbackCB, this, _1));
    
    visualization_msgs::InteractiveMarker int_goal_marker;
    int_goal_marker.header.frame_id = odom_frame_id_;
    lleg_goal_pose_ = leg_poses->midcoords() * getDefaultLeftLegOffset();
    rleg_goal_pose_ = leg_poses->midcoords() * getDefaultRightLegOffset();
    setupGoalMarker(leg_poses->midcoords(), int_goal_marker);
    if (is_2d_mode_) {
      add3Dof2DControl(int_goal_marker, false);
    }
    else {
      im_helpers::add6DofControl(int_goal_marker, false);
    }
    
    server_->insert(int_goal_marker,
                    boost::bind(&FootstepMarker::processFeedbackCB, this, _1));
    menu_handler_.apply(*server_, "movable_footstep_marker");
    menu_handler_.apply(*server_, "initial_footstep_marker");
    server_->applyChanges();
    updateMarkerArray(int_marker.header, int_goal_marker.pose);
  }

  void FootstepMarker::setupMenuHandler()
  {
    menu_handler_.insert("Reset Marker", boost::bind(&FootstepMarker::resetMarkerCB, this, _1));
    menu_handler_.insert("Execute Footstep", boost::bind(&FootstepMarker::executeFootstepCB, this, _1));
    interactive_markers::MenuHandler::EntryHandle mode_handle = menu_handler_.insert("2D/3D Mode");
    entry_2d_mode_ = menu_handler_.insert(mode_handle, "2D mode", boost::bind(&FootstepMarker::enable2DCB, this, _1));    
    entry_3d_mode_ = menu_handler_.insert(mode_handle, "3D mode", boost::bind(&FootstepMarker::enable3DCB, this, _1));
    if (is_2d_mode_) {
      menu_handler_.setCheckState(entry_2d_mode_,
                                  interactive_markers::MenuHandler::CHECKED);
      menu_handler_.setCheckState(entry_3d_mode_,
                                interactive_markers::MenuHandler::UNCHECKED);
    }
    else {
      menu_handler_.setCheckState(entry_2d_mode_,
                                  interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.setCheckState(entry_3d_mode_,
                                interactive_markers::MenuHandler::CHECKED);
    }

    interactive_markers::MenuHandler::EntryHandle vis_mode_handle = menu_handler_.insert("Visualization");
    cube_mode_ = menu_handler_.insert(vis_mode_handle, "Cube", boost::bind(&FootstepMarker::enableCubeCB, this, _1));
    line_mode_ = menu_handler_.insert(vis_mode_handle, "Line", boost::bind(&FootstepMarker::enableLineCB, this, _1));
    if (is_cube_mode_) {
      menu_handler_.setCheckState(cube_mode_, interactive_markers::MenuHandler::CHECKED);
      menu_handler_.setCheckState(line_mode_, interactive_markers::MenuHandler::UNCHECKED);
    }
    else {
      menu_handler_.setCheckState(cube_mode_, interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.setCheckState(line_mode_, interactive_markers::MenuHandler::CHECKED);
    }
  }

  void FootstepMarker::resetMarkerCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    resetInteractiveMarker();
  }

  void FootstepMarker::executeDoneCB(const actionlib::SimpleClientGoalState &state,
                                     const ExecResult::ConstPtr &result)
  {
    ROS_INFO("Done footsteps");
    resetInteractiveMarker();
  }
  
  void FootstepMarker::executeFootstepCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    // Lock footstep planner
    boost::mutex::scoped_lock lock(planner_mutex_);
    if (planning_state_ == FINISHED) {
      jsk_footstep_msgs::ExecFootstepsGoal goal;
      goal.footstep = plan_result_;
      ROS_INFO("Execute footsteps");
      ac_exec_.sendGoal(goal, boost::bind(&FootstepMarker::executeDoneCB, this, _1, _2));
      ac_exec_.waitForResult(ros::Duration(120.0));
    }
    else if (planning_state_ == ON_GOING) {
      ROS_FATAL("cannot execute footstep because planning state is ON_GOING");
    }
    else if (planning_state_ == NOT_STARTED) {
      ROS_FATAL("cannot execute footstep because planning state is NOT_STARTED");
    }
  }

  
  void FootstepMarker::enableCubeCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    interactive_markers::MenuHandler::CheckState check_state;
    menu_handler_.getCheckState(cube_mode_, check_state);
    if (check_state == interactive_markers::MenuHandler::UNCHECKED) {
      menu_handler_.setCheckState(cube_mode_, interactive_markers::MenuHandler::CHECKED);
      menu_handler_.setCheckState(line_mode_, interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.reApply(*server_);
      is_cube_mode_ = true;
      resetInteractiveMarker();
    }
  }

  void FootstepMarker::enableLineCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    interactive_markers::MenuHandler::CheckState check_state;
    menu_handler_.getCheckState(line_mode_, check_state);
    if (check_state == interactive_markers::MenuHandler::UNCHECKED) {
      menu_handler_.setCheckState(cube_mode_, interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.setCheckState(line_mode_, interactive_markers::MenuHandler::CHECKED);
      menu_handler_.reApply(*server_);
      is_cube_mode_ = false;
      resetInteractiveMarker();
    }
  }
  
  void FootstepMarker::enable2DCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    interactive_markers::MenuHandler::CheckState check_state;
    menu_handler_.getCheckState(entry_2d_mode_, check_state);
    if (check_state == interactive_markers::MenuHandler::UNCHECKED) {
      menu_handler_.setCheckState(entry_2d_mode_, interactive_markers::MenuHandler::CHECKED);
      menu_handler_.setCheckState(entry_3d_mode_, interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.reApply(*server_);
      // remove 6dof marker and use 3dof marker
      is_2d_mode_ = true;
      resetInteractiveMarker();
    }
    
  }

  void FootstepMarker::enable3DCB(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    interactive_markers::MenuHandler::CheckState check_state;
    menu_handler_.getCheckState(entry_3d_mode_, check_state);
    if (check_state == interactive_markers::MenuHandler::UNCHECKED) {
      menu_handler_.setCheckState(entry_3d_mode_, interactive_markers::MenuHandler::CHECKED);
      menu_handler_.setCheckState(entry_2d_mode_, interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.reApply(*server_);
      // remove 3dof marker and use 6dof marker
      is_2d_mode_ = false;
      resetInteractiveMarker();
    }
  }
  
  void FootstepMarker::processMenuFeedbackCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    
  }

  void FootstepMarker::cancelPlanning()
  {
    boost::mutex::scoped_lock lock(planner_mutex_);
    if (planning_state_ == ON_GOING) {
      ac_planner_.cancelGoal();
      planning_state_ = FINISHED;
    }
  }

  jsk_footstep_msgs::FootstepArray FootstepMarker::footstepArrayFromPosePair(PosePair::Ptr pose_pair,
                                                                             const std_msgs::Header& header,
                                                                             bool is_lleg_first)
  {
    jsk_footstep_msgs::FootstepArray footstep_array;
    footstep_array.header = header;
    jsk_footstep_msgs::Footstep lleg = footstepFromEigenPose(pose_pair->getByName(lleg_end_coords_));
    jsk_footstep_msgs::Footstep rleg = footstepFromEigenPose(pose_pair->getByName(rleg_end_coords_));
    lleg.leg = jsk_footstep_msgs::Footstep::LEFT;
    rleg.leg = jsk_footstep_msgs::Footstep::RIGHT;
    lleg.dimensions.x = foot_size_x_;
    lleg.dimensions.y = foot_size_y_;
    lleg.dimensions.z = foot_size_z_;
    rleg.dimensions.x = foot_size_x_;
    rleg.dimensions.y = foot_size_y_;
    rleg.dimensions.z = foot_size_z_;
    if (is_lleg_first) {
      footstep_array.footsteps.push_back(lleg);
      footstep_array.footsteps.push_back(rleg);
    }
    else {
      footstep_array.footsteps.push_back(rleg);
      footstep_array.footsteps.push_back(lleg);
    }
    return footstep_array;
  }
  
  void FootstepMarker::plan(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    std_msgs::Header header;
    header.frame_id = odom_frame_id_;
    header.stamp = ros::Time::now();
    // first, project footstep using footstep_planner/project_footprint_with_local_search API.
    jsk_interactive_marker::SnapFootPrint srv_arg;
    
    PosePair::Ptr goal_pose_pair(new PosePair(lleg_goal_pose_, lleg_end_coords_,
                                              rleg_goal_pose_, rleg_end_coords_));
    srv_arg.request.input_pose.header = header;
    Eigen::Affine3f midcoords = goal_pose_pair->midcoords();
    Eigen::Affine3f lleg_trans = lleg_goal_pose_ * midcoords.inverse();
    Eigen::Affine3f rleg_trans = rleg_goal_pose_ * midcoords.inverse();
    tf::poseEigenToMsg(midcoords, srv_arg.request.input_pose.pose);
    tf::poseEigenToMsg(lleg_trans, srv_arg.request.lleg_pose);
    tf::poseEigenToMsg(rleg_trans, srv_arg.request.rleg_pose);
    if (ros::service::call("footstep_planner/project_footprint_with_local_search", srv_arg)) {
      if (srv_arg.response.success) {
        Eigen::Affine3f new_center_pose;
        tf::poseMsgToEigen(srv_arg.response.snapped_pose.pose, new_center_pose);
        goal_pose_pair.reset(new PosePair(new_center_pose * getDefaultLeftLegOffset(), lleg_end_coords_,
                                          new_center_pose * getDefaultRightLegOffset(), rleg_end_coords_));
      }
      else {
        ROS_ERROR("Failed to project goal");
        return;
      }
    }
    jsk_footstep_msgs::PlanFootstepsGoal goal;
    goal.goal_footstep
      = footstepArrayFromPosePair(goal_pose_pair, header, true);
    goal.initial_footstep
      = footstepArrayFromPosePair(original_foot_poses_, header, true);
    
    ac_planner_.sendGoal(goal, boost::bind(&FootstepMarker::planDoneCB, this, _1, _2));
    planning_state_ = ON_GOING;
  }

  void FootstepMarker::planDoneCB(const actionlib::SimpleClientGoalState &state,
                                  const PlanResult::ConstPtr &result)
  {
    boost::mutex::scoped_lock lock(planner_mutex_);
    pub_plan_result_.publish(result->result);
    planning_state_ = FINISHED;
    plan_result_ = result->result;
  }
  
  void FootstepMarker::planIfPossible(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    boost::mutex::scoped_lock lock(planner_mutex_);
    if (planning_state_ != ON_GOING) {
      plan(feedback);
    }
  }

  Eigen::Affine3f FootstepMarker::getDefaultLeftLegOffset() {
    return Eigen::Affine3f(Eigen::Translation3f(0, default_footstep_margin_ / 2.0, 0.0));
  }

  Eigen::Affine3f FootstepMarker::getDefaultRightLegOffset() {
    return Eigen::Affine3f(Eigen::Translation3f(0, - default_footstep_margin_ / 2.0, 0.0));
  }
  
  void FootstepMarker::processFeedbackCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP) {
      cancelPlanning();
      plan(feedback);
    }
    else if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN) {
      
    }
    else if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
      // update position of goal footstep
      Eigen::Affine3f current_marker_pose;
      tf::poseMsgToEigen(feedback->pose, current_marker_pose);
      lleg_goal_pose_ = current_marker_pose * getDefaultLeftLegOffset();
      rleg_goal_pose_ = current_marker_pose * getDefaultRightLegOffset();
      planIfPossible(feedback);
    }
    updateMarkerArray(feedback->header, feedback->pose);
  }

  visualization_msgs::Marker FootstepMarker::targetArrow(
    const std_msgs::Header& header, const geometry_msgs::Pose& pose)
  {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.1;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.pose = pose;
    return marker;
  }
  
  visualization_msgs::Marker FootstepMarker::distanceLineMarker(
    const std_msgs::Header& header, const geometry_msgs::Pose& pose)
  {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 24 / 255.0;
    marker.color.g = 240 / 255.0;
    marker.color.b = 1.0;
    Eigen::Affine3f origin = original_foot_poses_->midcoords();
    Eigen::Affine3f posef;
    tf::poseMsgToEigen(pose, posef);
    Eigen::Vector3f direction(posef.translation() - origin.translation());
    Eigen::Vector3f normalized_direction = direction.normalized();
    Eigen::Vector3f original_x_direction = origin.rotation() * Eigen::Vector3f::UnitX();
    Eigen::Vector3f rotate_axis = original_x_direction.cross(normalized_direction).normalized();
    double pose_theta = acos(original_x_direction.dot(normalized_direction));
    Eigen::Affine3f transform;
    if (pose_theta == 0.0) {
      transform = origin * Eigen::Affine3f::Identity();
    }
    else {
      //transform = origin * Eigen::Translation3f(-origin.translation()) *  Eigen::AngleAxisf(pose_theta, rotate_axis);
      transform = origin * Eigen::AngleAxisf(pose_theta, rotate_axis);
    }
    double distance = (origin.inverse() * posef).translation().norm();
    double r = distance / (2.0 * M_PI);
    // (x, y) = r(theta - sin,  1 - cos)
    const size_t resolustion = 100;
    const double max_z = 1.0;
    const double z_ratio = max_z / 2.0 / r;
    for (size_t i = 0; i < resolustion - 1; i++) {
      double theta = 2.0 * M_PI / resolustion * i;
      double next_theta = 2.0 * M_PI / resolustion * (i + 1);
      Eigen::Vector3f p = transform * Eigen::Vector3f(r * (theta - sin(theta)), 0, r * (1.0 - cos(theta)) * z_ratio);
      Eigen::Vector3f q = transform * Eigen::Vector3f(r * (next_theta - sin(next_theta)), 0, r * (1.0 - cos(next_theta)) * z_ratio);
      geometry_msgs::Point ros_p;
      ros_p.x = p[0];
      ros_p.y = p[1];
      ros_p.z = p[2];
      geometry_msgs::Point ros_q;
      ros_q.x = q[0];
      ros_q.y = q[1];
      ros_q.z = q[2];
      marker.colors.push_back(marker.color);
      marker.points.push_back(ros_p);
      marker.colors.push_back(marker.color);
      marker.points.push_back(ros_q);
    }
    return marker;
  }

  visualization_msgs::Marker FootstepMarker::originMarker(
    const std_msgs::Header& header, const geometry_msgs::Pose& pose)
  {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 24 / 255.0;
    marker.color.g = 240 / 255.0;
    marker.color.b = 1.0;
    Eigen::Affine3f origin = original_foot_poses_->midcoords();
    tf::poseEigenToMsg(origin, marker.pose);
    const size_t resolution = 100;
    const double r = 0.5;
    for (size_t i = 0; i < resolution + 1; i++) {
      double theta = 2.0 * M_PI / resolution * i;
      Eigen::Vector3f p(r * cos(theta), r * sin(theta), 0.0);
      geometry_msgs::Point ros_p;
      ros_p.x = p[0];
      ros_p.y = p[1];
      ros_p.z = p[2];
      marker.points.push_back(ros_p);
      marker.colors.push_back(marker.color);
    }
    return marker;
  }
  
  visualization_msgs::Marker FootstepMarker::distanceTextMarker(
    const std_msgs::Header& header, const geometry_msgs::Pose& pose)
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    Eigen::Affine3f posef;
    tf::poseMsgToEigen(pose, posef);
    Eigen::Affine3f text_pose = posef * Eigen::Translation3f(-0.1, 0, 0.1);
    Eigen::Affine3f transform = original_foot_poses_->midcoords().inverse() * posef;
    Eigen::Vector3f pos(transform.translation());
    float roll, pitch, yaw;
    pcl::getEulerAngles(transform, roll, pitch, yaw);
    marker.text = (boost::format("pos[m] = (%.2f, %.2f, %.2f)\nrot[deg] = (%.2f, %.2f, %.2f)\n%.2f [m]\n%.0f [deg]")
                   % (pos[0]) % (pos[1]) % (pos[2])
                   % (angles::to_degrees(roll)) % (angles::to_degrees(pitch)) % (angles::to_degrees(yaw))
                   % (pos.norm()) % (Eigen::AngleAxisf(transform.rotation()).angle() / M_PI * 180)).str();
    //marker.pose = pose;
    tf::poseEigenToMsg(text_pose, marker.pose);
    marker.header = header;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    return marker;
  }

  visualization_msgs::Marker FootstepMarker::originBoundingBoxMarker(
      const std_msgs::Header& header, const geometry_msgs::Pose& pose)
  {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = collision_bbox_size_[0];
    marker.scale.y = collision_bbox_size_[1];
    marker.scale.z = collision_bbox_size_[2];
    marker.color.a = 0.3;
    marker.color.r = 1.0;
    Eigen::Affine3f box_pose = original_foot_poses_->midcoords() * collision_bbox_offset_;
    tf::poseEigenToMsg(box_pose, marker.pose);
    return marker;
  }

  visualization_msgs::Marker FootstepMarker::goalBoundingBoxMarker(
      const std_msgs::Header& header, const geometry_msgs::Pose& pose)
  {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = collision_bbox_size_[0];
    marker.scale.y = collision_bbox_size_[1];
    marker.scale.z = collision_bbox_size_[2];
    marker.color.a = 0.3;
    marker.color.r = 1.0;
    Eigen::Affine3f input_pose;
    tf::poseMsgToEigen(pose, input_pose);
    Eigen::Affine3f box_pose = input_pose * collision_bbox_offset_;
    tf::poseEigenToMsg(box_pose, marker.pose);
    return marker;
  }
  

  void FootstepMarker::updateMarkerArray(const std_msgs::Header& header, const geometry_msgs::Pose& pose)
  {
    pub_marker_array_.insert("target arrow", targetArrow(header, pose));
    pub_marker_array_.insert("distance text", distanceTextMarker(header, pose));
    pub_marker_array_.insert("distance line", distanceLineMarker(header, pose));
    pub_marker_array_.insert("origin", originMarker(header, pose));
    pub_marker_array_.insert("origin_bbox", originBoundingBoxMarker(header, pose));
    pub_marker_array_.insert("goal_bbox", goalBoundingBoxMarker(header, pose));
    pub_marker_array_.publish();
  }

  void FootstepMarker::processPoseUpdateCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
  }

  PosePair::Ptr FootstepMarker::getDefaultFootstepPair()
  {
    Eigen::Affine3f lleg_default_pose(Eigen::Translation3f(0, 0.1, 0) * Eigen::Translation3f(lleg_footstep_offset_));
    Eigen::Affine3f rleg_default_pose(Eigen::Translation3f(0, -0.1, 0) * Eigen::Translation3f(rleg_footstep_offset_));
    return PosePair::Ptr(new PosePair(lleg_default_pose, lleg_end_coords_,
                                      rleg_default_pose, rleg_end_coords_));
  }
  
  PosePair::Ptr FootstepMarker::getLatestCurrentFootstepPoses()
  {
    while (ros::ok())
    {
      try
      {
        return getCurrentFootstepPoses(ros::Time(0));
      }
      catch (tf2::TransformException& e)
      {
        JSK_ROS_WARN("tf error, retry: %s", e.what());
      }
    }
  }

  PosePair::Ptr FootstepMarker::getCurrentFootstepPoses(const ros::Time& stamp)
  {
    // use tf2
    // odom -> lleg_end_coords
    geometry_msgs::TransformStamped lleg_transform
      = tf_client_->lookupTransform(odom_frame_id_, lleg_end_coords_, stamp);
    geometry_msgs::TransformStamped rleg_transform
      = tf_client_->lookupTransform(odom_frame_id_, rleg_end_coords_, stamp);
    Eigen::Affine3f lleg_transform_eigen, rleg_transform_eigen;
    tf::transformMsgToEigen(lleg_transform.transform, lleg_transform_eigen);
    tf::transformMsgToEigen(rleg_transform.transform, rleg_transform_eigen);
    return PosePair::Ptr(new PosePair(lleg_transform_eigen * Eigen::Translation3f(lleg_footstep_offset_), lleg_end_coords_,
                                      rleg_transform_eigen * Eigen::Translation3f(rleg_footstep_offset_), rleg_end_coords_));
  }

  void FootstepMarker::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(planner_mutex_);
    disable_tf_ = config.disable_tf;
    default_footstep_margin_ = config.default_footstep_margin;
  }


  void FootstepMarker::poseStampedCommandCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    JSK_ROS_DEBUG("posestamped command is received");
    std_msgs::Header tmp_header = msg->header;
    tmp_header.frame_id = odom_frame_id_;
    {
      // apply target pose to goal marker
      // mutex should be limited in this range because planIfPossible also lock planner_mutex_
      boost::mutex::scoped_lock lock(planner_mutex_);
      Eigen::Affine3f eigen_command_pose;
      server_->setPose("movable_footstep_marker", msg->pose, tmp_header);
      server_->applyChanges();
    }
    // forcely call processFeedbackCB to execute planning
    visualization_msgs::InteractiveMarkerFeedback dummy_feedback;
    dummy_feedback.header = tmp_header;
    dummy_feedback.pose = msg->pose;
    dummy_feedback.event_type = visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE;
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr dummy_feedback_ptr
      = boost::make_shared<const visualization_msgs::InteractiveMarkerFeedback>(dummy_feedback);
    processFeedbackCB(dummy_feedback_ptr);
  }
  
  bool FootstepMarker::resetMarkerService(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    // forcely call resetMarkerCB to reset marker. feedback msg does not used in restMarkerCB.
    visualization_msgs::InteractiveMarkerFeedback dummy_feedback;
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr dummy_feedback_ptr
      = boost::make_shared<const visualization_msgs::InteractiveMarkerFeedback>(dummy_feedback);
    resetMarkerCB(dummy_feedback_ptr);
    return true;
  }

  bool FootstepMarker::executeFootstepService(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    // forcely call executeFootstepCB to execute footstep. feedback msg does not used in executeFootstepCB.
    visualization_msgs::InteractiveMarkerFeedback dummy_feedback;
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr dummy_feedback_ptr
      = boost::make_shared<const visualization_msgs::InteractiveMarkerFeedback>(dummy_feedback);
    executeFootstepCB(dummy_feedback_ptr);
    return true;
    // check footstep result
    if (ac_exec_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else {
      return false;
    }
  }

  bool FootstepMarker::getFootstepMarkerPoseService(
    jsk_interactive_marker::GetTransformableMarkerPose::Request& req,
    jsk_interactive_marker::GetTransformableMarkerPose::Response& res)
  {
    boost::mutex::scoped_lock lock(planner_mutex_);
    std::string target_name = req.target_name;
    visualization_msgs::InteractiveMarker int_marker;
    if (server_->get(target_name, int_marker)) {
      geometry_msgs::PoseStamped ret_pose_stamped;
      ret_pose_stamped.header = int_marker.header;
      ret_pose_stamped.pose = int_marker.pose;
      res.pose_stamped = ret_pose_stamped;
      return true;
    } else {
      ROS_WARN("There is no marker named %s", target_name.c_str());
      return false;
    }
  }
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "footstep_marker");
  jsk_footstep_planner::FootstepMarker marker;
  ros::spin();
}
