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


#ifndef JSK_FOOTSTEP_PLANNER_FOOTSTEP_SOLVER_H_
#define JSK_FOOTSTEP_PLANNER_FOOTSTEP_SOLVER_H_

#include "jsk_footstep_planner/astar_solver.h"
#include "jsk_footstep_planner/footstep_state_discrete_close_list.h"

namespace jsk_footstep_planner
{
  // Only available for FootstepState and FootstepGraph
  // because close list behavior is specialized for the purpose
  template <class GraphT>
  class FootstepAStarSolver: public AStarSolver<GraphT>
  {
  public:
    typedef boost::shared_ptr<FootstepAStarSolver> Ptr;
    typedef typename GraphT::StateT State;
    typedef typename GraphT::StateT::Ptr StatePtr;
    typedef typename GraphT::Ptr GraphPtr;
    typedef typename SolverNode<State, GraphT>::Ptr SolverNodePtr;
    typedef typename boost::function<void(FootstepAStarSolver&, GraphPtr)> ProfileFunction;
    typedef typename std::priority_queue<SolverNodePtr,
                                         std::vector<SolverNodePtr>,
                                         std::greater<SolverNodePtr> > OpenList;    
    FootstepAStarSolver(
      GraphPtr graph, size_t x_num, size_t y_num, size_t theta_num,
      unsigned int profile_period = 1024,
      double cost_weight = 1.0,
      double heuristic_weight = 1.0):
      footstep_close_list_(x_num, y_num, theta_num),
      profile_period_(profile_period),
      is_set_profile_function_(false),
      cost_weight_(cost_weight),
      heuristic_weight_(heuristic_weight),
      is_cancelled_(false),
      AStarSolver<GraphT>(graph)
    {
      
    }

    virtual double fn(SolverNodePtr n)
    {
      return cost_weight_ * gn(n) + heuristic_weight_ * hn(n);
    }

    
    virtual
    std::vector<typename SolverNode<State, GraphT>::Ptr>
    solve(const ros::WallDuration& timeout = ros::WallDuration(1000000000.0))
    {
      ros::WallTime start_time = ros::WallTime::now();
      SolverNodePtr start_state(new SolverNode<State, GraphT>(
                                  graph_->getStartState(),
                                  0, graph_));
      TransitionLimit::Ptr limit = graph_->getTransitionLimit();
      bool lazy_projection = graph_->lazyProjection();
      addToOpenList(start_state);
      while (!is_cancelled_ && !isOpenListEmpty()  && isOK(start_time, timeout)) {
        SolverNodePtr target_node = popFromOpenList();
        if (graph_->usePointCloudModel() && lazy_projection) {
          unsigned int error_state;
          FootstepState::Ptr projected_state = graph_->projectFootstep(target_node->getState(),
                                                                       error_state);
          if (!projected_state) { // failed to project footstep
            if (graph_->localMovement() && error_state == projection_state::close_to_success) {
              // try local movement
              std::vector<FootstepState::Ptr> locally_moved_states;
              {
                std::vector<FootstepState::Ptr> states_candidates
                  = graph_->localMoveFootstepState(target_node->getState());
                for (int i = 0; i < states_candidates.size(); i ++) {
                  FootstepGraph::StatePtr tmp_state = graph_->projectFootstep(states_candidates[i],
                                                                              error_state);
                  if (!!tmp_state) {
                    locally_moved_states.push_back(tmp_state);
                  }
                }
              }
              if (locally_moved_states.size() > 0) {
                std::vector<SolverNodePtr> locally_moved_nodes
                  = target_node->wrapWithSolverNodes(target_node->getParent(),
                                                     locally_moved_states);
                for (size_t i = 0; i < locally_moved_nodes.size(); i++) {

                  if (graph_->isSuccessable(locally_moved_nodes[i]->getState(),
                                            target_node->getParent()->getState())) {
                    addToOpenList(locally_moved_nodes[i]);
                  }
                }
              }
            }
            continue;           // back to the top of while loop
          }
          else {
            if (target_node->getParent()) {
              if (graph_->isSuccessable(projected_state, target_node->getParent()->getState())) {
                target_node->setState(projected_state);
              }
              else {
                continue;
              }
            }
            else {
              target_node->setState(projected_state);
            }
          }
        } //if (graph_->usePointCloudModel() && lazy_projection) {
        if (graph_->isGoal(target_node->getState())) {
          if (is_set_profile_function_) {
            profile_function_(*this, graph_);
          }
          std::vector<SolverNodePtr> result_path = target_node->getPathWithoutThis();
          result_path.push_back(target_node);
          return result_path;
        }
        else if (!findInCloseList(target_node->getState())) {
          addToCloseList(target_node->getState());
          std::vector<SolverNodePtr> next_nodes = target_node->expand(target_node, verbose_);
          // Add to open list only if next_nodes is not in close list.
          // We can do it thanks to FootstepStateDiscreteCloseList
          for (size_t i = 0; i < next_nodes.size(); i++) {
            SolverNodePtr next_node = next_nodes[i];
            if (!findInCloseList(next_node->getState())) {
              addToOpenList(next_node);
            }
          }
        }
      }
      if (is_cancelled_) {
        ROS_WARN("FootstepAStarSolver is cancelled");
      }
      // Failed to search
      return std::vector<SolverNodePtr>();
    }
    
    virtual void cancelSolve() {
      is_cancelled_ = true;
      ROS_FATAL("cancel planning");
    }

    // Overtake closelist behavior from solver class
    virtual bool findInCloseList(StatePtr s)
    {
      return footstep_close_list_.find(s);
    }

    virtual void addToCloseList(StatePtr s)
    {
      footstep_close_list_.push_back(s);
    }

    virtual OpenList getOpenList() { return open_list_; }
    
    virtual FootstepStateDiscreteCloseList getCloseList() { return footstep_close_list_; }
    virtual void setProfileFunction(ProfileFunction f)
    {
      profile_function_ = f;
      is_set_profile_function_ = true;
    }

    virtual bool isOK(const ros::WallTime& start_time, const ros::WallDuration& timeout)
    {
      if (is_set_profile_function_ && ++loop_counter_ % profile_period_ == 0) {
        profile_function_(*this, graph_);
        loop_counter_ = 0;
      }
      return (ros::ok() && (ros::WallTime::now() - start_time) < timeout);
    }

    template <class PointT>
    void closeListToPointCloud(pcl::PointCloud<PointT>& output_cloud)
    {
      footstep_close_list_.toPointCloud<PointT>(output_cloud);
    }
    
    template <class PointT>
    void openListToPointCloud(pcl::PointCloud<PointT>& output_cloud)
    {
      output_cloud.points.reserve(open_list_.size());
      OpenList copied_open_list = open_list_;
      
      while (copied_open_list.size() > 0)
      {
        SolverNodePtr solver_node = copied_open_list.top();
        StatePtr state = solver_node->getState();
        PointT p = ((FootstepState::Ptr)state)->toPoint<PointT>(); // hacky way
        output_cloud.points.push_back(p);
        copied_open_list.pop();
      }
    }

    void addToOpenList(SolverNodePtr node)
    {
      if (node->isRoot()) {
        BestFirstSearchSolver<GraphT>::addToOpenList(node);
      }
      else {
        if (node->getState()->crossCheck(
              node->getParent()->getState())) {
          BestFirstSearchSolver<GraphT>::addToOpenList(node);
        }
      }
    }
    
    using Solver<GraphT>::isOpenListEmpty;
    using Solver<GraphT>::popFromOpenList;
    using Solver<GraphT>::addToOpenList;
    using BestFirstSearchSolver<GraphT>::addToOpenList;
    using AStarSolver<GraphT>::gn;
    using AStarSolver<GraphT>::hn;
    
  protected:
    unsigned int loop_counter_;
    unsigned int profile_period_;
    ProfileFunction profile_function_;
    bool is_set_profile_function_;
    FootstepStateDiscreteCloseList footstep_close_list_;
    using Solver<GraphT>::graph_;
    using Solver<GraphT>::verbose_;
    using BestFirstSearchSolver<GraphT>::open_list_;
    const double cost_weight_;
    const double heuristic_weight_;
    bool is_cancelled_;
  };
}

#endif
