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
#include <dynamic_reconfigure/Reconfigure.h>

#define printAffine(af) { \
  geometry_msgs::Pose __geom_pose;\
  tf::poseEigenToMsg(af, __geom_pose);\
  std::cerr << __geom_pose.position.x << ", ";\
  std::cerr << __geom_pose.position.y << ", ";\
  std::cerr << __geom_pose.position.z << " / ";\
  std::cerr << __geom_pose.orientation.w << ", ";\
  std::cerr << __geom_pose.orientation.x << ", ";\
  std::cerr << __geom_pose.orientation.y << ", ";\
  std::cerr << __geom_pose.orientation.z << std::endl; }

namespace jsk_recognition_utils
{
  void convertEigenAffine3(const Eigen::Affine3f& from,
                           Eigen::Affine3f& to)
  {
    to = from;
  }
}

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

  PosePair::PosePair(const FootstepTrans& first, const std::string& first_name,
                     const FootstepTrans& second, const std::string& second_name):
    first_(first), first_name_(first_name),
    second_(second), second_name_(second_name)
  {

  }

  FootstepTrans PosePair::getByName(const std::string& name)
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

  FootstepTrans PosePair::midcoords()
  {
    FootstepTranslation pos((FootstepVec(first_.translation()) + FootstepVec(second_.translation())) / 2.0);
    FootstepQuaternion rot = FootstepQuaternion(first_.rotation()).slerp(0.5, FootstepQuaternion(second_.rotation()));
    return pos * rot;
  }

  FootstepMarker::FootstepMarker():
    pnh_("~"), ac_planner_("footstep_planner", true), ac_exec_("footstep_controller", true),
    pub_marker_array_(pnh_, "marker_array"),
    is_2d_mode_(true), is_cube_mode_(false), command_mode_(SINGLE), have_last_step_(false),
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
    srv_reset_fs_marker_    = pnh_.advertiseService("reset_marker",
                                                    &FootstepMarker::resetMarkerService, this);
    srv_toggle_fs_com_mode_ = pnh_.advertiseService("toggle_footstep_marker_mode",
                                                    &FootstepMarker::toggleFootstepMarkerModeService, this);
    srv_execute_footstep_   = pnh_.advertiseService("execute_footstep",
                                                    &FootstepMarker::executeFootstepService, this);
    srv_wait_for_exec_fs_   = pnh_.advertiseService("wait_for_execute",
                                                    &FootstepMarker::waitForExecuteFootstepService, this);
    srv_wait_for_fs_plan_   = pnh_.advertiseService("wait_for_plan",
                                                    &FootstepMarker::waitForFootstepPlanService, this);
    srv_get_fs_marker_pose_ = pnh_.advertiseService("get_footstep_marker_pose",
                                                    &FootstepMarker::getFootstepMarkerPoseService, this);
    srv_stack_marker_pose_  = pnh_.advertiseService("stack_marker_pose",
                                                    &FootstepMarker::stackMarkerPoseService, this);

    pub_plan_result_ = pnh_.advertise<jsk_footstep_msgs::FootstepArray>("output/plan_result", 1);
    pub_current_marker_mode_ = pnh_.advertise<jsk_rviz_plugins::OverlayText>("marker_mode", 1, true);

    //ROS_INFO("waiting for footstep_planner");
    //ac_planner_.waitForServer();
    //ROS_INFO("waiting for footstep_controller");
    //ac_exec_.waitForServer();
    // initialize interactive marker
    // build menu handler
    setupMenuHandler();
    resetInteractiveMarker();
    publishCurrentMarkerMode();
    ROS_INFO("initialization done");
  }

  FootstepMarker::~FootstepMarker()
  {
    pub_marker_array_.clear();
    pub_marker_array_.publish();
    ros::Duration(1.0).sleep();
  }

  visualization_msgs::Marker FootstepMarker::makeFootstepMarker(FootstepTrans pose, unsigned char leg)
  {
    FootstepTrans footpose = pose;
    if (leg == jsk_footstep_msgs::Footstep::LLEG) {
      FootstepTranslation ltrans(lleg_footstep_offset_[0], lleg_footstep_offset_[1], lleg_footstep_offset_[2]);
      footpose = pose * ltrans;
    } else if (leg == jsk_footstep_msgs::Footstep::RLEG) {
      FootstepTranslation rtrans(rleg_footstep_offset_[0], rleg_footstep_offset_[1], rleg_footstep_offset_[2]);
      footpose = pose * rtrans;
    } else {
      ROS_ERROR ("makeFootstepMarker not implemented leg (%d)", leg);
    }
    visualization_msgs::Marker marker;
    if (is_cube_mode_) {
      marker.type = visualization_msgs::Marker::CUBE;
      marker.scale.x = foot_size_x_;
      marker.scale.y = foot_size_y_;
      marker.scale.z = foot_size_z_;
      tf::poseEigenToMsg(footpose, marker.pose);
    }
    else {
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.scale.x = 0.01;
      FootstepVec local_a( foot_size_x_ / 2.0,  foot_size_y_ / 2.0, 0.0);
      FootstepVec local_b(-foot_size_x_ / 2.0,  foot_size_y_ / 2.0, 0.0);
      FootstepVec local_c(-foot_size_x_ / 2.0, -foot_size_y_ / 2.0, 0.0);
      FootstepVec local_d( foot_size_x_ / 2.0, -foot_size_y_ / 2.0, 0.0);
      FootstepVec a = footpose * local_a;
      FootstepVec b = footpose * local_b;
      FootstepVec c = footpose * local_c;
      FootstepVec d = footpose * local_d;
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
    FootstepTrans midcoords = leg_poses->midcoords();
    tf::poseEigenToMsg(midcoords, int_marker.pose);
    visualization_msgs::Marker left_box_marker = makeFootstepMarker(midcoords.inverse() * leg_poses->getByName(lleg_end_coords_),
                                                                    jsk_footstep_msgs::Footstep::LLEG);
    left_box_marker.color.g = 1.0;
    visualization_msgs::Marker right_box_marker = makeFootstepMarker(midcoords.inverse() * leg_poses->getByName(rleg_end_coords_),
                                                                     jsk_footstep_msgs::Footstep::RLEG);
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

  void FootstepMarker::setupGoalMarker(FootstepTrans pose,
                                       visualization_msgs::InteractiveMarker& int_goal_marker)
  {
    int_goal_marker.name = "movable_footstep_marker";
    int_goal_marker.description = "Goal Footsteps";
    tf::poseEigenToMsg(pose, int_goal_marker.pose);
    current_lleg_offset_ = pose.inverse() * lleg_goal_pose_;
    current_rleg_offset_ = pose.inverse() * rleg_goal_pose_;
    visualization_msgs::Marker left_box_marker = makeFootstepMarker(current_lleg_offset_, jsk_footstep_msgs::Footstep::LLEG);
    left_box_marker.color.g = 1.0;
    visualization_msgs::Marker right_box_marker = makeFootstepMarker(current_rleg_offset_, jsk_footstep_msgs::Footstep::RLEG);
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
    ROS_INFO("reset marker");
    PosePair::Ptr leg_poses;
    if (disable_tf_) {
      leg_poses = getDefaultFootstepPair();
    }
    else {
      if(command_mode_ == SINGLE || !have_last_step_) {
        leg_poses = getLatestCurrentFootstepPoses();
      } else { // !single_mode && have_last_step_
        FootstepTrans lleg_pose;
        FootstepTrans rleg_pose;
        tf::poseMsgToEigen(last_steps_[0].pose, lleg_pose);
        tf::poseMsgToEigen(last_steps_[1].pose, rleg_pose);
        leg_poses =
          PosePair::Ptr(new PosePair(lleg_pose, lleg_end_coords_,
                                     rleg_pose, rleg_end_coords_));
      }
    }
    original_foot_poses_ = leg_poses;
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = odom_frame_id_;

    setupInitialMarker(leg_poses, int_marker);
    server_->insert(int_marker,
                    boost::bind(&FootstepMarker::processFeedbackCB, this, _1));

    visualization_msgs::InteractiveMarker int_goal_marker;
    int_goal_marker.header.frame_id = odom_frame_id_;
    lleg_goal_pose_ = leg_poses->getByName(lleg_end_coords_);
    rleg_goal_pose_ = leg_poses->getByName(rleg_end_coords_);
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
    stack_btn_ = menu_handler_.insert("Stack Footstep", boost::bind(&FootstepMarker::stackFootstepCB, this, _1));
    menu_handler_.setVisible(stack_btn_, false);
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

    interactive_markers::MenuHandler::EntryHandle p_mode_handle = menu_handler_.insert("Plan Mode");
    single_mode_ = menu_handler_.insert(p_mode_handle, "Single", boost::bind(&FootstepMarker::enableSingleCB, this, _1));
    cont_mode_ = menu_handler_.insert(p_mode_handle, "Continuous", boost::bind(&FootstepMarker::enableContinuousCB, this, _1));
    stack_mode_ = menu_handler_.insert(p_mode_handle, "Stack", boost::bind(&FootstepMarker::enableStackCB, this, _1)); // TODO
    menu_handler_.setCheckState(single_mode_, interactive_markers::MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(cont_mode_,   interactive_markers::MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(stack_mode_,  interactive_markers::MenuHandler::UNCHECKED);
    switch(command_mode_) {
    case SINGLE:
      menu_handler_.setCheckState(single_mode_, interactive_markers::MenuHandler::CHECKED);
      break;
    case CONTINUOUS:
      menu_handler_.setCheckState(cont_mode_,   interactive_markers::MenuHandler::CHECKED);
      break;
    case STACK:
      stacked_poses_.resize(0);
      stacked_poses_.push_back(original_foot_poses_->midcoords());
      menu_handler_.setVisible(stack_btn_, true);
      menu_handler_.setCheckState(stack_mode_,  interactive_markers::MenuHandler::CHECKED);
      break;
    }
  }

  void FootstepMarker::resetMarkerCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    actionlib::SimpleClientGoalState state = ac_exec_.getState();
    if (state.isDone()) { // Do not reset last step while footsteps are executed
      have_last_step_ = false;
    }
    resetInteractiveMarker();
  }

  void FootstepMarker::executeDoneCB(const actionlib::SimpleClientGoalState &state,
                                     const ExecResult::ConstPtr &result)
  {
    ROS_INFO("Done footsteps");
    resetInteractiveMarker();
  }

  void FootstepMarker::stackFootstepCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    PosePair::Ptr goal_pose_pair(new PosePair(lleg_goal_pose_, lleg_end_coords_,
                                              rleg_goal_pose_, rleg_end_coords_));
    FootstepTrans midcoords = goal_pose_pair->midcoords();
    stacked_poses_.push_back(midcoords);

    updateMarkerArray(feedback->header, feedback->pose);
  }

  void FootstepMarker::executeFootstepCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    // Lock footstep planner
    boost::mutex::scoped_lock lock(planner_mutex_);
    if (planning_state_ == FINISHED) {
      jsk_footstep_msgs::ExecFootstepsGoal goal;
      planning_state_ = NOT_STARTED;
      goal.footstep = plan_result_;
      if (command_mode_ == SINGLE) {
        //if(ac_exec_.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
        //  ac_exec_.waitForResult(ros::Duration(120.0));
        //  return;
        //}
        goal.strategy = jsk_footstep_msgs::ExecFootstepsGoal::NEW_TARGET;
        have_last_step_ = false;

        ROS_INFO("Execute footsteps single");
        ac_exec_.sendGoal(goal, boost::bind(&FootstepMarker::executeDoneCB, this, _1, _2));
      } else { // continuous mode
        if (have_last_step_ ) {
          goal.strategy = jsk_footstep_msgs::ExecFootstepsGoal::RESUME;
        } else {
          goal.strategy = jsk_footstep_msgs::ExecFootstepsGoal::NEW_TARGET;
        }

        int size = plan_result_.footsteps.size();
        {
          if(plan_result_.footsteps[size-1].leg == jsk_footstep_msgs::Footstep::LEFT) {
            last_steps_[0] = plan_result_.footsteps[size-1]; // left
            last_steps_[1] = plan_result_.footsteps[size-2]; // right
          } else {
            last_steps_[0] = plan_result_.footsteps[size-2]; // left
            last_steps_[1] = plan_result_.footsteps[size-1]; // right
          }
        }
        have_last_step_ = true;
        if (goal.strategy == jsk_footstep_msgs::ExecFootstepsGoal::NEW_TARGET) {
          ROS_INFO("Execute footsteps continuous(new)");
        } else {
          ROS_INFO("Execute footsteps continuous(added)");
        }
        // wait result or ...
        if (ac_exec_.isServerConnected()) {
          ac_exec_.sendGoal(goal, boost::bind(&FootstepMarker::executeDoneCB, this, _1, _2));
        } else {
          ROS_FATAL("actionlib server is not connected");
        }
        resetInteractiveMarker();
      }
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

  void FootstepMarker::enableSingleCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    interactive_markers::MenuHandler::CheckState check_state;
    menu_handler_.getCheckState(single_mode_, check_state);
    if (check_state == interactive_markers::MenuHandler::UNCHECKED) {
      menu_handler_.setVisible(stack_btn_, false);
      menu_handler_.setCheckState(single_mode_, interactive_markers::MenuHandler::CHECKED);
      menu_handler_.setCheckState(cont_mode_,   interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.setCheckState(stack_mode_,  interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.reApply(*server_);
      command_mode_ = SINGLE;
      resetInteractiveMarker();
      publishCurrentMarkerMode();
      { // change heuristic
        dynamic_reconfigure::Reconfigure rconf;
        dynamic_reconfigure::StrParameter spara;
        spara.name = "heuristic";
        spara.value = "path_cost";
        rconf.request.config.strs.push_back(spara);
        if (!ros::service::call("footstep_planner/set_parameters", rconf)) {
          // ERROR
          ROS_ERROR("Dynamic reconfigure: set parameters failed");
          return;
        }
      }
    }
  }

  void FootstepMarker::enableContinuousCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    interactive_markers::MenuHandler::CheckState check_state;
    menu_handler_.getCheckState(cont_mode_, check_state);
    if (check_state == interactive_markers::MenuHandler::UNCHECKED) {
      menu_handler_.setVisible(stack_btn_, false);
      menu_handler_.setCheckState(single_mode_, interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.setCheckState(cont_mode_,   interactive_markers::MenuHandler::CHECKED);
      menu_handler_.setCheckState(stack_mode_,  interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.reApply(*server_);
      command_mode_ = CONTINUOUS;
      resetInteractiveMarker();
      publishCurrentMarkerMode();
      { // change heuristic
        dynamic_reconfigure::Reconfigure rconf;
        dynamic_reconfigure::StrParameter spara;
        spara.name = "heuristic";
        spara.value = "path_cost";
        rconf.request.config.strs.push_back(spara);
        if (!ros::service::call("footstep_planner/set_parameters", rconf)) {
          // ERROR
          ROS_ERROR("Dynamic reconfigure: set parameters failed");
          return;
        }
      }
    }
  }

  void FootstepMarker::enableStackCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    interactive_markers::MenuHandler::CheckState check_state;
    menu_handler_.getCheckState(stack_mode_, check_state);
    if (check_state == interactive_markers::MenuHandler::UNCHECKED) {
      menu_handler_.setVisible(stack_btn_, true);
      menu_handler_.setCheckState(single_mode_, interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.setCheckState(cont_mode_,   interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.setCheckState(stack_mode_,  interactive_markers::MenuHandler::CHECKED);
      menu_handler_.reApply(*server_);
      command_mode_ = STACK;
      stacked_poses_.resize(0);
      stacked_poses_.push_back(original_foot_poses_->midcoords());
      resetInteractiveMarker();
      publishCurrentMarkerMode();
      { // change heuristic
        dynamic_reconfigure::Reconfigure rconf;
        dynamic_reconfigure::StrParameter spara;
        spara.name = "heuristic";
        spara.value = "follow_path";
        rconf.request.config.strs.push_back(spara);
        if (!ros::service::call("footstep_planner/set_parameters", rconf)) {
          // ERROR
          ROS_ERROR("Dynamic reconfigure: set parameters failed");
          return;
        }
      }
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
    FootstepTrans midcoords = goal_pose_pair->midcoords();
    FootstepTrans lleg_trans = midcoords.inverse() * lleg_goal_pose_;
    FootstepTrans rleg_trans = midcoords.inverse() * rleg_goal_pose_;
    tf::poseEigenToMsg(midcoords, srv_arg.request.input_pose.pose);
    tf::poseEigenToMsg(lleg_trans, srv_arg.request.lleg_pose);
    tf::poseEigenToMsg(rleg_trans, srv_arg.request.rleg_pose);
    if (ros::service::call("footstep_planner/project_footprint_with_local_search", srv_arg)) {
      if (srv_arg.response.success) {
        FootstepTrans new_center_pose;
        tf::poseMsgToEigen(srv_arg.response.snapped_pose.pose, new_center_pose);
        goal_pose_pair.reset(new PosePair(new_center_pose * lleg_trans, lleg_end_coords_,
                                          new_center_pose * rleg_trans, rleg_end_coords_));
      }
      else {
        ROS_ERROR("Failed to project goal"); // should display message
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
      ROS_INFO("start planning");
      plan(feedback);
    }
  }

  FootstepTrans FootstepMarker::getDefaultLeftLegOffset() {
    return FootstepTrans(FootstepTranslation(0, default_footstep_margin_ / 2.0, 0));
  }

  FootstepTrans FootstepMarker::getDefaultRightLegOffset() {
    return FootstepTrans(FootstepTranslation(0, - default_footstep_margin_ / 2.0, 0));
  }

  void FootstepMarker::processFeedbackCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP) {
      ROS_INFO("mouse_up");
      if(command_mode_ != STACK) {
        ROS_INFO("cancel and planning");
        cancelPlanning();
        plan(feedback);
      } else {
        FootstepTrans current_marker_pose;
        tf::poseMsgToEigen(feedback->pose, current_marker_pose);
        lleg_goal_pose_ = current_marker_pose * current_lleg_offset_;
        rleg_goal_pose_ = current_marker_pose * current_rleg_offset_;
      }
    }
    else if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN) {

    }
    else if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
      ROS_INFO("pose_up");
      // update position of goal footstep
      FootstepTrans current_marker_pose;
      tf::poseMsgToEigen(feedback->pose, current_marker_pose);
      lleg_goal_pose_ = current_marker_pose * current_lleg_offset_;
      rleg_goal_pose_ = current_marker_pose * current_rleg_offset_;
      if(command_mode_ != STACK) {
        planIfPossible(feedback);
      } else { // stack mode
        if (planning_state_ != ON_GOING) {
          ROS_INFO("follow plan");
          callFollowPathPlan(feedback);
        }
      }
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
    FootstepTrans origin = original_foot_poses_->midcoords();
    FootstepTrans posef;
    tf::poseMsgToEigen(pose, posef);
    FootstepVec direction(posef.translation() - origin.translation());
    FootstepVec normalized_direction = direction.normalized();
    FootstepVec original_x_direction = origin.rotation() * FootstepVec::UnitX();
    FootstepVec rotate_axis = original_x_direction.cross(normalized_direction).normalized();
    double pose_theta = acos(original_x_direction.dot(normalized_direction));
    FootstepTrans transform;
    if (pose_theta == 0.0) {
      transform = origin * FootstepTrans::Identity();
    }
    else {
      //transform = origin * FootstepTranslation(-origin.translation()) *  Eigen::AngleAxisf(pose_theta, rotate_axis);
      transform = origin * FootstepAngleAxis(pose_theta, rotate_axis);
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
      FootstepVec p = transform * FootstepVec(r * (theta - sin(theta)), 0, r * (1.0 - cos(theta)) * z_ratio);
      FootstepVec q = transform * FootstepVec(r * (next_theta - sin(next_theta)), 0, r * (1.0 - cos(next_theta)) * z_ratio);
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
    FootstepTrans origin = original_foot_poses_->midcoords();
    tf::poseEigenToMsg(origin, marker.pose);
    const size_t resolution = 100;
    const double r = 0.5;
    for (size_t i = 0; i < resolution + 1; i++) {
      double theta = 2.0 * M_PI / resolution * i;
      FootstepVec p(r * cos(theta), r * sin(theta), 0.0);
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
    Eigen::Affine3f midcoords;
    jsk_recognition_utils::convertEigenAffine3(original_foot_poses_->midcoords().inverse(), midcoords);
    Eigen::Affine3f transform = midcoords * posef;
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
    FootstepTrans box_pose = original_foot_poses_->midcoords() * collision_bbox_offset_;
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
    FootstepTrans input_pose;
    tf::poseMsgToEigen(pose, input_pose);
    FootstepTrans box_pose = input_pose * collision_bbox_offset_;
    tf::poseEigenToMsg(box_pose, marker.pose);
    return marker;
  }

  visualization_msgs::Marker FootstepMarker::stackedPosesMarker(
    const std_msgs::Header& header, const geometry_msgs::Pose& pose)
  {
    visualization_msgs::Marker marker;
    marker.header = header;
    if (command_mode_ != STACK) {
      return marker;
    }
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = 0.025; // line width
    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 1.0;
    col.g = 240 / 255.0;
    col.b = 24 / 255.0;
    marker.color = col;

    for(int i=1; i < stacked_poses_.size(); i++) {
      FootstepVec p = stacked_poses_[i-1].translation();
      FootstepVec q = stacked_poses_[i].translation();
      geometry_msgs::Point ros_p;
      ros_p.x = p[0];
      ros_p.y = p[1];
      ros_p.z = p[2];
      geometry_msgs::Point ros_q;
      ros_q.x = q[0];
      ros_q.y = q[1];
      ros_q.z = q[2];
      marker.colors.push_back(col);
      marker.points.push_back(ros_p);
      marker.colors.push_back(col);
      marker.points.push_back(ros_q);
    }
    // add current
    FootstepVec p = stacked_poses_[stacked_poses_.size() - 1].translation();
    geometry_msgs::Point ros_p;
    ros_p.x = p[0];
    ros_p.y = p[1];
    ros_p.z = p[2];
    geometry_msgs::Point ros_q;
    ros_q.x = pose.position.x;
    ros_q.y = pose.position.y;
    ros_q.z = pose.position.z;
    col.r = 1.0;
    col.g = 80 / 255.0;
    col.b = 80 / 255.0;
    marker.colors.push_back(col);
    marker.points.push_back(ros_p);
    marker.colors.push_back(col);
    marker.points.push_back(ros_q);

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
    pub_marker_array_.insert("stacked_pose", stackedPosesMarker(header, pose));
    pub_marker_array_.publish();
  }

  void FootstepMarker::processPoseUpdateCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
  }

  PosePair::Ptr FootstepMarker::getDefaultFootstepPair()
  {
    FootstepTrans lleg_default_pose(FootstepTranslation(0, 0.1, 0));
    FootstepTrans rleg_default_pose(FootstepTranslation(0, -0.1, 0));
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
        ROS_WARN("tf error, retry: %s", e.what());
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
    FootstepTrans lleg_transform_eigen, rleg_transform_eigen;
    tf::transformMsgToEigen(lleg_transform.transform, lleg_transform_eigen);
    tf::transformMsgToEigen(rleg_transform.transform, rleg_transform_eigen);
    PosePair::Ptr ppair (new PosePair(lleg_transform_eigen, lleg_end_coords_,
                                      rleg_transform_eigen, rleg_end_coords_));
    if(use_default_goal_) {
      return PosePair::Ptr(new PosePair(ppair->midcoords() * getDefaultLeftLegOffset(),  lleg_end_coords_,
                                        ppair->midcoords() * getDefaultRightLegOffset(), rleg_end_coords_));
    }
    return ppair;
  }

  void FootstepMarker::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(planner_mutex_);
    disable_tf_ = config.disable_tf;
    default_footstep_margin_ = config.default_footstep_margin;
    use_default_goal_ =  config.use_default_step_as_goal;
  }


  void FootstepMarker::poseStampedCommandCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    ROS_DEBUG("posestamped command is received");
    ROS_INFO("posestamped command is received");
    geometry_msgs::PoseStamped tmp_pose_stamped;
    if(msg->header.frame_id != odom_frame_id_ && !disable_tf_) {
      // apply target pose to goal marker
      // mutex should be limited in this range because planIfPossible also lock planner_mutex_
      boost::mutex::scoped_lock lock(planner_mutex_);
      try {
        geometry_msgs::TransformStamped offset = tf_client_->lookupTransform(odom_frame_id_, msg->header.frame_id,
                                                                             msg->header.stamp, ros::Duration(2.0));
        FootstepTrans msg_eigen;
        FootstepTrans offset_eigen;
        tf::poseMsgToEigen(msg->pose, msg_eigen);
        tf::transformMsgToEigen(offset.transform, offset_eigen);
        FootstepTrans pose = msg_eigen * offset_eigen;

        tf::poseEigenToMsg(pose, tmp_pose_stamped.pose);
        tmp_pose_stamped.header.stamp = msg->header.stamp;
        tmp_pose_stamped.header.frame_id = odom_frame_id_;
      } catch(tf2::TransformException ex) {
        ROS_ERROR("posestamped command transformation failed %s",ex.what());
        return;
      }
    } else {
      tmp_pose_stamped = *msg;
    }
    server_->setPose("movable_footstep_marker", tmp_pose_stamped.pose, tmp_pose_stamped.header);
    server_->applyChanges();
    // forcely call processFeedbackCB to execute planning
    visualization_msgs::InteractiveMarkerFeedback dummy_feedback;
    dummy_feedback.header = tmp_pose_stamped.header;
    dummy_feedback.pose = tmp_pose_stamped.pose;
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

  bool FootstepMarker::toggleFootstepMarkerModeService(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    visualization_msgs::InteractiveMarkerFeedback dummy_feedback;
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr dummy_feedback_ptr
      = boost::make_shared<const visualization_msgs::InteractiveMarkerFeedback>(dummy_feedback);

    if (command_mode_ == SINGLE) {
      // single -> continuous
      enableContinuousCB(dummy_feedback_ptr);
      menu_handler_.setCheckState(single_mode_, interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.setCheckState(cont_mode_, interactive_markers::MenuHandler::CHECKED);
      menu_handler_.setCheckState(stack_mode_, interactive_markers::MenuHandler::UNCHECKED);
    } else if (command_mode_ == CONTINUOUS) {
      // continuous -> stack
      enableStackCB(dummy_feedback_ptr);
      menu_handler_.setCheckState(single_mode_, interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.setCheckState(cont_mode_, interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.setCheckState(stack_mode_, interactive_markers::MenuHandler::CHECKED);
    } else {
      // stack -> single
      enableSingleCB(dummy_feedback_ptr);
      menu_handler_.setCheckState(single_mode_, interactive_markers::MenuHandler::CHECKED);
      menu_handler_.setCheckState(cont_mode_, interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.setCheckState(stack_mode_, interactive_markers::MenuHandler::UNCHECKED);
    }
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

  bool FootstepMarker::waitForExecuteFootstepService(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    bool result = true;
    actionlib::SimpleClientGoalState state = ac_exec_.getState();
    if(!state.isDone()) {
      result = ac_exec_.waitForResult(ros::Duration(120.0));
    }
    return result;
  }

  bool FootstepMarker::waitForFootstepPlanService(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    bool result = true;
    actionlib::SimpleClientGoalState state = ac_planner_.getState();
    if(!state.isDone()) {
      result = ac_planner_.waitForResult(ros::Duration(120.0));
    }
    return result;
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

  bool FootstepMarker::stackMarkerPoseService(
      jsk_interactive_marker::SetPose::Request& req,
      jsk_interactive_marker::SetPose::Response& res)
  {
    ROS_INFO("stack marker service");
    if (command_mode_ != STACK) {
      return false;
    }
    visualization_msgs::InteractiveMarkerFeedback dummy_feedback;
    dummy_feedback.header = req.pose.header;
    dummy_feedback.pose   = req.pose.pose;
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr dummy_feedback_ptr
      = boost::make_shared<const visualization_msgs::InteractiveMarkerFeedback>(dummy_feedback);
    stackFootstepCB(dummy_feedback_ptr);
    return true;
  }

  void FootstepMarker::publishCurrentMarkerMode()
  {
    std_msgs::ColorRGBA color;
    color.r = 0.3568627450980392;
    color.g = 0.7529411764705882;
    color.b = 0.8705882352941177;
    color.a = 1.0;

    std::string text;
    switch(command_mode_) {
    case SINGLE:
      text = "Single Mode";
      break;
    case CONTINUOUS:
      text = "Continuous Mode";
      break;
    case STACK:
      text = "Stack Mode";
      break;
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
    pub_current_marker_mode_.publish(msg);
  }

  void FootstepMarker::callFollowPathPlan(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    if(stacked_poses_.size() > 1) {
      boost::mutex::scoped_lock lock(planner_mutex_);

      jsk_footstep_planner::SetHeuristicPath srv_arg;
      for(int i = 0; i < stacked_poses_.size(); i++) {
        geometry_msgs::Point p;
        FootstepVec tl = stacked_poses_[i].translation();
        p.x = tl[0];
        p.y = tl[1];
        p.z = tl[2];
        srv_arg.request.segments.push_back(p);
      }
      { // add final pose
        geometry_msgs::Point p;
        PosePair::Ptr goal_pose_pair(new PosePair(lleg_goal_pose_, lleg_end_coords_,
                                                  rleg_goal_pose_, rleg_end_coords_));
        FootstepVec tl = goal_pose_pair->midcoords().translation();
        p.x = tl[0];
        p.y = tl[1];
        p.z = tl[2];
        srv_arg.request.segments.push_back(p);
      }
      if (!ros::service::call("footstep_planner/set_heuristic_path", srv_arg)) {
        // ERROR
        ROS_ERROR("Service: failed to call footstep_planner/set_heuristic_path");
        return;
      }
    } else {
      return;
    }
    // plan
    plan(feedback);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "footstep_marker");
  jsk_footstep_planner::FootstepMarker marker;
  ros::spin();
}
