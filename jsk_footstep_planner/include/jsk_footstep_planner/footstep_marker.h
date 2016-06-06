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


#ifndef JSK_FOOTSTEP_PLANNER_FOOTSTEP_MARKER_H_
#define JSK_FOOTSTEP_PLANNER_FOOTSTEP_MARKER_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <jsk_footstep_msgs/PlanFootstepsAction.h>
#include <jsk_footstep_msgs/ExecFootstepsAction.h>
#include <jsk_interactive_marker/GetTransformableMarkerPose.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <interactive_markers/menu_handler.h>
#include <tf2_ros/buffer_client.h>
#include <Eigen/Geometry>
#include <visualization_msgs/MarkerArray.h>
#include "jsk_footstep_planner/marker_array_publisher.h"
#include "jsk_footstep_planner/FootstepMarkerConfig.h"
#include <dynamic_reconfigure/server.h>

namespace jsk_footstep_planner
{

  class UnknownPoseName: public std::exception
  {

  };
  
  class PosePair
  {
  public:
    typedef boost::shared_ptr<PosePair> Ptr;
    PosePair(const Eigen::Affine3f& first, const std::string& first_name,
             const Eigen::Affine3f& second, const std::string& second_name);
    virtual Eigen::Affine3f getByName(const std::string& name);
    virtual Eigen::Affine3f midcoords();
  protected:
    Eigen::Affine3f first_;
    Eigen::Affine3f second_;
    std::string first_name_;
    std::string second_name_;
  private:
    
  };
  
  class FootstepMarker
  {
  public:
    typedef boost::shared_ptr<FootstepMarker> Ptr;
    typedef actionlib::SimpleActionClient<jsk_footstep_msgs::PlanFootstepsAction>
    PlanningActionClient;
    typedef actionlib::SimpleActionClient<jsk_footstep_msgs::ExecFootstepsAction>
    ExecuteActionClient;
    typedef jsk_footstep_msgs::PlanFootstepsResult PlanResult;
    typedef jsk_footstep_msgs::ExecFootstepsResult ExecResult;
    typedef boost::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)>
    MenuCallbackFunction;

    typedef enum {NOT_STARTED, FINISHED, ON_GOING} PlanningState;
    typedef FootstepMarkerConfig Config;
    FootstepMarker();
    virtual ~FootstepMarker();
  protected:
    virtual void resetInteractiveMarker();
    virtual PosePair::Ptr getLatestCurrentFootstepPoses();
    virtual PosePair::Ptr getCurrentFootstepPoses(const ros::Time& stamp);
    virtual PosePair::Ptr getDefaultFootstepPair();
    virtual visualization_msgs::Marker makeFootstepMarker(Eigen::Affine3f pose);
    virtual void processFeedbackCB(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    virtual void processMenuFeedbackCB(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    virtual void processPoseUpdateCB(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    virtual void resetMarkerCB(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    virtual void enable2DCB(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    virtual void enable3DCB(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    virtual void enableCubeCB(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    virtual void enableLineCB(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    virtual void executeFootstepCB(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    virtual void executeDoneCB(const actionlib::SimpleClientGoalState &state,
                               const ExecResult::ConstPtr &result);
    virtual Eigen::Affine3f getDefaultLeftLegOffset();
    virtual Eigen::Affine3f getDefaultRightLegOffset();
    // planner interface
    virtual void cancelPlanning();
    virtual void plan(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    virtual void planIfPossible(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    virtual void planDoneCB(const actionlib::SimpleClientGoalState &state,
                            const PlanResult::ConstPtr &result);
    virtual jsk_footstep_msgs::FootstepArray footstepArrayFromPosePair(PosePair::Ptr pose_pair,
                                                                       const std_msgs::Header& header,
                                                                       bool is_lleg_first);
    // marker methods
    virtual void setupInitialMarker(PosePair::Ptr leg_poses,
                                    visualization_msgs::InteractiveMarker& int_marker);
    virtual void setupGoalMarker(Eigen::Affine3f pose,
                                 visualization_msgs::InteractiveMarker& int_marker);
    virtual void updateMarkerArray(const std_msgs::Header& header, const geometry_msgs::Pose& pose);
    virtual visualization_msgs::Marker originMarker(
      const std_msgs::Header& header, const geometry_msgs::Pose& pose);
    virtual visualization_msgs::Marker distanceTextMarker(
      const std_msgs::Header& header, const geometry_msgs::Pose& pose);
    virtual visualization_msgs::Marker distanceLineMarker(
      const std_msgs::Header& header, const geometry_msgs::Pose& pose);
    virtual visualization_msgs::Marker targetArrow(
      const std_msgs::Header& header, const geometry_msgs::Pose& pose);
    virtual visualization_msgs::Marker originBoundingBoxMarker(
      const std_msgs::Header& header, const geometry_msgs::Pose& pose);
    virtual visualization_msgs::Marker goalBoundingBoxMarker(
      const std_msgs::Header& header, const geometry_msgs::Pose& pose);
    virtual void setupMenuHandler();

    virtual void configCallback(Config& config, uint32_t level);
    virtual void poseStampedCommandCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    virtual bool resetMarkerService(
      std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res);
    virtual bool executeFootstepService(
      std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res);
    virtual bool getFootstepMarkerPoseService(
      jsk_interactive_marker::GetTransformableMarkerPose::Request& req,
      jsk_interactive_marker::GetTransformableMarkerPose::Response& res);
    
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    MarkerArrayPublisher pub_marker_array_;
    PlanningActionClient ac_planner_;
    ExecuteActionClient ac_exec_;
    ros::Publisher pub_plan_result_;
    ros::Subscriber sub_pose_stamped_command_;
    ros::ServiceServer srv_reset_marker_;
    ros::ServiceServer srv_execute_footstep_;
    ros::ServiceServer srv_get_footstep_marker_pose_;    
    
    std::string odom_frame_id_;
    std::string lleg_end_coords_, rleg_end_coords_;
    PosePair::Ptr original_foot_poses_;
    Eigen::Affine3f lleg_goal_pose_, rleg_goal_pose_;
    Eigen::Vector3f lleg_footstep_offset_, rleg_footstep_offset_;
    double default_footstep_margin_;
    
    jsk_footstep_msgs::FootstepArray plan_result_;
    boost::shared_ptr<tf2_ros::BufferClient> tf_client_;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    interactive_markers::MenuHandler menu_handler_;
    interactive_markers::MenuHandler::EntryHandle entry_2d_mode_;
    interactive_markers::MenuHandler::EntryHandle entry_3d_mode_;
    interactive_markers::MenuHandler::EntryHandle cube_mode_;
    interactive_markers::MenuHandler::EntryHandle line_mode_;
    bool is_2d_mode_;
    bool is_cube_mode_;
    
    double foot_size_x_, foot_size_y_, foot_size_z_;
    bool disable_tf_;
    
    boost::mutex planner_mutex_;
    PlanningState planning_state_;
    Eigen::Vector3f collision_bbox_size_;
    Eigen::Affine3f collision_bbox_offset_;
  private:
    
  };
}

#endif
