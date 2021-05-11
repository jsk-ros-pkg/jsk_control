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


#ifndef JSK_FOOTSTEP_PLANNER_FOOTSTEP_GRAPH_H_
#define JSK_FOOTSTEP_PLANNER_FOOTSTEP_GRAPH_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <jsk_footstep_msgs/FootstepArray.h>

#include "jsk_recognition_utils/geo_util.h"
#include "jsk_footstep_planner/graph.h"
#include "jsk_footstep_planner/footstep_state.h"
#include "jsk_footstep_planner/astar_solver.h"
#include "jsk_footstep_planner/ann_grid.h"
#include "jsk_footstep_planner/transition_limit.h"
#include "jsk_footstep_planner/footstep_parameters.h"

namespace jsk_footstep_planner
{
  class FootstepGraph: public Graph<FootstepState>
  {
  public:
    typedef boost::shared_ptr<FootstepGraph> Ptr;
    typedef typename boost::function< bool(StatePtr target_state, std::vector<StatePtr> &) > SuccessorFunction;
    typedef typename boost::function< double(StatePtr, StatePtr, double) > PathCostFunction;

    friend double footstepHeuristicFollowPathLine(SolverNode<FootstepState, FootstepGraph>::Ptr node,
                                                  FootstepGraph::Ptr graph);
    FootstepGraph(const Eigen::Vector3f& resolution,
                  const bool use_pointcloud_model = false,
                  const bool lazy_projection = true,
                  const bool local_movement = false,
                  const bool use_obstacle_model = false):
      max_successor_distance_(0.0), max_successor_rotation_(0.0),
      publish_progress_(false),
      resolution_(resolution),
      use_pointcloud_model_(use_pointcloud_model),
      lazy_projection_(lazy_projection),
      local_movement_(local_movement),
      use_obstacle_model_(use_obstacle_model),
      pointcloud_model_2d_(new pcl::PointCloud<pcl::PointNormal>),
      tree_model_(new pcl::KdTreeFLANN<pcl::PointNormal>),
      obstacle_tree_model_(new pcl::KdTreeFLANN<pcl::PointXYZ>),
      tree_model_2d_(new pcl::search::Octree<pcl::PointNormal>(0.2)),
      grid_search_(new ANNGrid(0.05)),
      parameters_(),
      zero_state_(new FootstepState(0,
                                    Eigen::Affine3f::Identity(),
                                    Eigen::Vector3f::UnitX(),
                                    resolution_)),
      perception_duration_(0.0)
    {
    }
    virtual std::vector<StatePtr> successors(StatePtr target_state) {
      std::vector<StatePtr> ret;
      successor_func_(target_state, ret);
      return ret;
    }

    virtual bool isGoal(StatePtr state);

    virtual double pathCost(StatePtr from, StatePtr to, double prev_cost)
    {
      return path_cost_func_(from, to, prev_cost);
    }

    /**
     * @brief
     * return True if current_state collides with obstacle.
     */
    virtual bool isColliding(StatePtr current_state, StatePtr previous_state);
    virtual pcl::PointIndices::Ptr getPointIndicesCollidingSphere(const Eigen::Affine3f& c);
    virtual bool isCollidingBox(const Eigen::Affine3f& c, pcl::PointIndices::Ptr candidates) const;
    /**
     * @brief
     * Compute robot coords from current footstep and previous footstep.
     * R_robot = midcoords(F_current, F_previous) * R_offset;
     */
    virtual Eigen::Affine3f getRobotCoords(StatePtr current_state, StatePtr previous_state) const;
    virtual void setBasicSuccessors(
      std::vector<Eigen::Affine3f> left_to_right_successors);
    
    virtual void setGoalState(
      FootstepState::Ptr left, FootstepState::Ptr right)
    {
      left_goal_state_ = left;
      right_goal_state_ = right;
    }

    virtual void setLeftGoalState(FootstepState::Ptr goal)
    {
      left_goal_state_ = goal;
    }
    
    virtual void setRightGoalState(FootstepState::Ptr goal)
    {
      right_goal_state_ = goal;
    }

    /**
     * @brief
     * return string about graph information.
     */
    virtual std::string infoString() const;
    
    bool finalizeSteps(const StatePtr &last_1_Step, const StatePtr &lastStep,
                       std::vector<StatePtr> &finalizeSteps);

    virtual FootstepState::Ptr getGoal(int leg)
    {
      if (leg == jsk_footstep_msgs::Footstep::LEFT) {
        return left_goal_state_;
      }
      else if (leg == jsk_footstep_msgs::Footstep::RIGHT) {
        return right_goal_state_;
      }
      else {                    // TODO: error
        return goal_state_;
      }
    }

    virtual double maxSuccessorDistance()
    {
      return max_successor_distance_;
    }
    virtual double maxSuccessorRotation()
    {
      return max_successor_rotation_;
    }

    virtual void setProgressPublisher(ros::NodeHandle& nh, std::string topic)
    {
      publish_progress_ = true;
      pub_progress_ = nh.advertise<jsk_footstep_msgs::FootstepArray>(topic, 1);
    }

    virtual void setPointCloudModel(pcl::PointCloud<pcl::PointNormal>::Ptr model)
    {
      pointcloud_model_ = model;
      tree_model_->setInputCloud(pointcloud_model_);
      // Project point_cloud_model_
      pcl::ProjectInliers<pcl::PointNormal> proj;
      proj.setModelType(pcl::SACMODEL_PLANE);
      pcl::ModelCoefficients::Ptr
        plane_coefficients (new pcl::ModelCoefficients);
      plane_coefficients->values.resize(4.0);
      plane_coefficients->values[2] = 1.0; // TODO: configurable
      proj.setModelCoefficients(plane_coefficients);
      proj.setInputCloud(pointcloud_model_);
      proj.filter(*pointcloud_model_2d_);
      tree_model_2d_->setInputCloud(pointcloud_model_2d_);
      grid_search_->build(*model);
    }

    virtual void setObstacleModel(pcl::PointCloud<pcl::PointXYZ>::Ptr model)
    {
      obstacle_model_ = model;
      obstacle_tree_model_->setInputCloud(obstacle_model_);
    }
    
    virtual bool projectGoal();
    virtual bool projectStart();
    virtual bool isSuccessable(StatePtr current_state, StatePtr previous_state);
    virtual bool useObstacleModel() const { return use_obstacle_model_; }
    virtual bool usePointCloudModel() const { return use_pointcloud_model_; }
    virtual bool lazyProjection()  const { return lazy_projection_; }
    virtual bool localMovement() const { return local_movement_; }

    virtual void setParameters (FootstepParameters &p) { parameters_ = p; }
    virtual void setTransitionLimit(TransitionLimit::Ptr limit) { transition_limit_ = limit; }
    virtual TransitionLimit::Ptr getTransitionLimit() { return transition_limit_; }
    virtual void setGlobalTransitionLimit(TransitionLimit::Ptr limit) { global_transition_limit_ = limit; }
    virtual TransitionLimit::Ptr getGlobalTransitionLimit() { return global_transition_limit_; }
    virtual FootstepState::Ptr projectFootstep(FootstepState::Ptr in);
    virtual FootstepState::Ptr projectFootstep(FootstepState::Ptr in, unsigned int& state);
    virtual ros::WallDuration getPerceptionDuration() { return perception_duration_; }
    virtual void clearPerceptionDuration() { perception_duration_ = ros::WallDuration(0.0); }
    virtual std::vector<FootstepState::Ptr> localMoveFootstepState(FootstepState::Ptr in);
    virtual void setCollisionBBoxOffset(const Eigen::Affine3f& offset) { collision_bbox_offset_ = offset; }
    virtual void setCollisionBBoxSize(const Eigen::Vector3f& size) { collision_bbox_size_ = size; }

    virtual void setSuccessorFunction(SuccessorFunction s) {
      successor_func_ = s;
    }
    virtual void setPathCostFunction(PathCostFunction p) {
      path_cost_func_ = p;
    }
    bool successors_original(StatePtr target_state, std::vector<FootstepGraph::StatePtr> &ret);
    double path_cost_original(StatePtr from, StatePtr to, double prev_cost) {
      // path cost is a number of steps
      return prev_cost + 1;
    }
    void setHeuristicPathLine(jsk_recognition_utils::PolyLine &path_line)
    {
      heuristic_path_.reset(new jsk_recognition_utils::PolyLine(path_line)); // copy ???
    }
  protected:
    pcl::PointCloud<pcl::PointNormal>::Ptr pointcloud_model_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_model_;
    pcl::PointCloud<pcl::PointNormal>::Ptr pointcloud_model_2d_;
    pcl::KdTreeFLANN<pcl::PointNormal>::Ptr tree_model_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr obstacle_tree_model_;
    //pcl::KdTreeFLANN<pcl::PointNormal>::Ptr tree_model_2d_;
    pcl::search::Octree<pcl::PointNormal>::Ptr tree_model_2d_;
    ANNGrid::Ptr grid_search_;
    std::vector<Eigen::Affine3f> successors_from_left_to_right_;
    std::vector<Eigen::Affine3f> successors_from_right_to_left_;
    FootstepState::Ptr left_goal_state_;
    FootstepState::Ptr right_goal_state_;
    /**
     * @brief
     * zero_state is used only for global transition limit
     */
    FootstepState::Ptr zero_state_;
    Eigen::Affine3f collision_bbox_offset_;
    Eigen::Vector3f collision_bbox_size_;
    double max_successor_distance_;
    double max_successor_rotation_;
    bool publish_progress_;
    ros::Publisher pub_progress_;
    // const params
    const bool use_pointcloud_model_;
    const bool lazy_projection_;
    const bool local_movement_;
    const bool use_obstacle_model_;
    const Eigen::Vector3f resolution_;
    // params
    TransitionLimit::Ptr transition_limit_;
    TransitionLimit::Ptr global_transition_limit_;
    FootstepParameters parameters_;

    ros::WallDuration perception_duration_;

    // add hint for huristic
    jsk_recognition_utils::PolyLine::Ptr heuristic_path_;
  private:
    SuccessorFunction successor_func_;
    PathCostFunction  path_cost_func_;
  };

  // heuristic function
  double footstepHeuristicZero(
    SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph);
  double footstepHeuristicStraight(
    SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph);
  double footstepHeuristicStraightRotation(
    SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph);
  double footstepHeuristicStepCost(
    SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph,
    double first_rotation_weight,
    double second_rotation_weight);
  double footstepHeuristicFollowPathLine(
    SolverNode<FootstepState, FootstepGraph>::Ptr node, FootstepGraph::Ptr graph);
}

#endif
