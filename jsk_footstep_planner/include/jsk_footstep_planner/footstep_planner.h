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


#ifndef JSK_FOOTSTEP_PLANNER_FOOTSTEP_PLANNER_H_
#define JSK_FOOTSTEP_PLANNER_FOOTSTEP_PLANNER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// ros
#include <jsk_footstep_msgs/FootstepArray.h>
#include <jsk_footstep_msgs/PlanFootstepsAction.h>
#include <jsk_footstep_planner/FootstepPlannerConfig.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_rviz_plugins/OverlayText.h>

// footstep planning
#include "jsk_footstep_planner/footstep_graph.h"
#include "jsk_footstep_planner/astar_solver.h"
#include "jsk_footstep_planner/footstep_astar_solver.h"
#include <jsk_footstep_planner/CollisionBoundingBoxInfo.h>
#include <jsk_interactive_marker/SnapFootPrint.h>

namespace jsk_footstep_planner
{

  enum PlanningStatus
  {
    OK, WARNING, ERROR
  };
  /**
   * @brief
   * Actionlib server for footstep planning
   */
  class FootstepPlanner
  {
  public:
    typedef boost::shared_ptr<FootstepPlanner> Ptr;
    typedef FootstepPlannerConfig Config;
    FootstepPlanner(ros::NodeHandle& nh);
  protected:

    // Initialization
    virtual bool readSuccessors(ros::NodeHandle& nh);
    
    virtual void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void planCB(const jsk_footstep_msgs::PlanFootstepsGoal::ConstPtr& goal);

    // buildGraph is not thread safe, it is responsible for caller to take care
    // of mutex
    virtual void buildGraph();

    virtual void configCallback(Config &config, uint32_t level);

    virtual double stepCostHeuristic(
      SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph);
    virtual double zeroHeuristic(
      SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph);
    virtual double straightHeuristic(
      SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph);
    virtual double straightRotationHeuristic(
      SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph);
    virtual void profile(FootstepAStarSolver<FootstepGraph>& solver, FootstepGraph::Ptr graph);
    virtual void publishPointCloud(
      const pcl::PointCloud<pcl::PointNormal>& cloud,
      ros::Publisher& pub,
      const std_msgs::Header& header);
    virtual bool projectFootPrint(
      const Eigen::Affine3f& center_pose,
      const Eigen::Affine3f& left_pose_trans,
      const Eigen::Affine3f& right_pose_trans,
      geometry_msgs::Pose& pose);

    virtual bool projectFootPrintService(
      jsk_interactive_marker::SnapFootPrint::Request& req,
      jsk_interactive_marker::SnapFootPrint::Response& res);
    virtual bool collisionBoundingBoxInfoService(
      jsk_footstep_planner::CollisionBoundingBoxInfo::Request& req,
      jsk_footstep_planner::CollisionBoundingBoxInfo::Response& res);
    virtual bool projectFootPrintWithLocalSearchService(
      jsk_interactive_marker::SnapFootPrint::Request& req,
      jsk_interactive_marker::SnapFootPrint::Response& res);
    virtual void publishText(ros::Publisher& pub,
                             const std::string& text,
                             PlanningStatus status);
    boost::mutex mutex_;
    actionlib::SimpleActionServer<jsk_footstep_msgs::PlanFootstepsAction> as_;
    jsk_footstep_msgs::PlanFootstepsResult result_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    ros::Publisher pub_close_list_;
    ros::Publisher pub_open_list_;
    ros::Publisher pub_text_;
    ros::Subscriber sub_pointcloud_model_;
    ros::Subscriber sub_obstacle_model_;
    ros::ServiceServer srv_project_footprint_;
    ros::ServiceServer srv_project_footprint_with_local_search_;
    ros::ServiceServer srv_collision_bounding_box_info_;
    pcl::PointCloud<pcl::PointNormal>::Ptr pointcloud_model_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_model_;
    FootstepGraph::Ptr graph_;
    std::vector<Eigen::Affine3f> successors_;
    Eigen::Vector3f collision_bbox_size_;
    Eigen::Affine3f collision_bbox_offset_;
    std_msgs::Header latest_header_;
    // Parameters
    bool rich_profiling_;
    bool use_pointcloud_model_;
    bool use_lazy_perception_;
    bool use_local_movement_;
    bool use_transition_limit_;
    bool use_obstacle_model_;
    bool use_global_transition_limit_;
    bool project_start_state_;
    bool project_goal_state_;
    double local_move_x_;
    double local_move_y_;
    double local_move_theta_;
    int local_move_x_num_;
    int local_move_y_num_;
    int local_move_theta_num_;
    double transition_limit_x_;
    double transition_limit_y_;
    double transition_limit_z_;
    double transition_limit_roll_;
    double transition_limit_pitch_;
    double transition_limit_yaw_;
    double global_transition_limit_roll_;
    double global_transition_limit_pitch_;
    double obstacle_resolution_;
    double goal_pos_thr_;
    double goal_rot_thr_;
    int plane_estimation_max_iterations_;
    int plane_estimation_min_inliers_;
    double plane_estimation_outlier_threshold_;
    int support_check_x_sampling_;
    int support_check_y_sampling_;
    double support_check_vertex_neighbor_threshold_;
    bool skip_cropping_;
    double resolution_x_;
    double resolution_y_;
    double resolution_theta_;
    double footstep_size_x_;
    double footstep_size_y_;
    int close_list_x_num_;
    int close_list_y_num_;
    int close_list_theta_num_;
    int profile_period_;
    std::string heuristic_;
    double heuristic_first_rotation_weight_;
    double heuristic_second_rotation_weight_;
    double cost_weight_;
    double heuristic_weight_;
    std::string pointcloud_model_frame_id_, obstacle_model_frame_id_;
  private:
    
  };
}

#endif
