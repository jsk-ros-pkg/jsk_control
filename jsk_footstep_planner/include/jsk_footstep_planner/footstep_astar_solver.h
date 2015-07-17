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
    FootstepAStarSolver(
      GraphPtr graph, size_t x_num, size_t y_num, size_t theta_num):
      footstep_close_list_(x_num, y_num, theta_num),
      AStarSolver<GraphT>(graph)
    {
      
    }

    virtual
    std::vector<typename SolverNode<State, GraphT>::Ptr>
    solve()
    {
      SolverNodePtr start_state(new SolverNode<State, GraphT>(
                                  graph_->getStartState(),
                                  0, graph_));
      bool lazy_projection = graph_->lazyProjection();
      addToOpenList(start_state);
      while (!isOpenListEmpty()) {
        SolverNodePtr target_node = popFromOpenList();
        if (graph_->usePointCloudModel() && lazy_projection) {
          FootstepState::Ptr projected_state = graph_->projectFootstep(target_node->getState());
          if (!target_node) {
            continue;
          }
          else {
            target_node->setState(projected_state);
          }
        }
        if (graph_->isGoal(target_node->getState())) {
          std::vector<SolverNodePtr> result_path = target_node->getPathWithoutThis();
          result_path.push_back(target_node);
          return result_path;
        }
        else if (!findInCloseList(target_node->getState())) {
          //close_list_.push_back(target_node->getState());
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
      // Failed to search
      return std::vector<SolverNodePtr>();
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
    using Solver<GraphT>::isOpenListEmpty;
    using Solver<GraphT>::popFromOpenList;
    using Solver<GraphT>::addToOpenList;
  protected:
    FootstepStateDiscreteCloseList footstep_close_list_;
    using Solver<GraphT>::graph_;
    using Solver<GraphT>::verbose_;

    
  };
}

#endif
