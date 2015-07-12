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


#ifndef JSK_FOOTSTEP_PLANNER_SOLVER_H_
#define JSK_FOOTSTEP_PLANNER_SOLVER_H_

#include "jsk_footstep_planner/graph.h"
#include "jsk_footstep_planner/solver_node.h"

namespace jsk_footstep_planner
{
  template <class GraphT>
  class Solver
  {
  public:
    typedef boost::shared_ptr<Solver> Ptr;
    typedef typename GraphT::StateT State;
    typedef typename GraphT::StateT::Ptr StatePtr;
    typedef typename GraphT::Ptr GraphPtr;
    typedef typename SolverNode<State, GraphT>::Ptr SolverNodePtr;
    Solver(): verbose_(false) {};
    Solver(GraphPtr graph): graph_(graph), verbose_(false) {}

    virtual void setVerbose(bool v) { verbose_ = v; }
    
    virtual
    std::vector<typename SolverNode<State, GraphT>::Ptr>
    solve()
    {
      SolverNodePtr start_state(new SolverNode<State, GraphT>(
                                  graph_->getStartState(),
                                  0, graph_));
      addToOpenList(start_state);
      while (!isOpenListEmpty()) {
        SolverNodePtr target_node = popFromOpenList();
        if (graph_->isGoal(target_node->getState())) {
          std::vector<SolverNodePtr> result_path = target_node->getPathWithoutThis();
          result_path.push_back(target_node);
          return result_path;
        }
        else if (!findInCloseList(target_node->getState())) {
          close_list_.push_back(target_node->getState());
          addToOpenList(target_node->expand(target_node, verbose_));
        }
      }
      // Failed to search
      return std::vector<SolverNodePtr>();
    }

    virtual bool isOpenListEmpty() = 0;
    virtual void addToOpenList(SolverNodePtr node) = 0;
    virtual SolverNodePtr popFromOpenList() = 0;
    
    virtual void addToOpenList(std::vector<SolverNodePtr> nodes)
    {
      for (size_t i = 0; i < nodes.size(); i++) {
        addToOpenList(nodes[i]);
      }
    }
    
    virtual bool findInCloseList(StatePtr state)
    {
      return std::find(close_list_.begin(), close_list_.end(), state)
        != close_list_.end();
    }

  protected:
    std::vector<StatePtr> close_list_;
    GraphPtr graph_;
    bool verbose_;
  private:
    
  };
}

#endif
