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


#ifndef JSK_FOOTSTEP_PLANNER_GRAPH_H_
#define JSK_FOOTSTEP_PLANNER_GRAPH_H_

#include <boost/shared_ptr.hpp>
#include <vector>

namespace jsk_footstep_planner
{
  template <class StateT_>
  class Graph
  {
  public:
    typedef boost::shared_ptr<Graph> Ptr;
    typedef StateT_ StateT;
    typedef boost::shared_ptr<StateT> StatePtr;
    Graph() {}
    virtual void setStartState(StatePtr start) { start_state_ = start; }
    virtual void setGoalState(StatePtr goal) { goal_state_ = goal; }

    virtual StatePtr getStartState() { return start_state_; }
    virtual StatePtr getGoalState() { return goal_state_; }
    
    virtual void addNode(StatePtr state) { nodes_.push_back(state); }
    virtual size_t numNodes() { return nodes_.size(); }
    virtual std::vector<StatePtr> successors(StatePtr target_state) = 0;
    virtual double pathCost(StatePtr from, StatePtr to, double prev_cost)
    {
      return prev_cost + 1;
    }
    
    virtual bool isGoal(StatePtr state) = 0;

  protected:
    StatePtr start_state_;
    StatePtr goal_state_;
    std::vector<StatePtr> nodes_;
    //double pos_goal_thr_;
    //double rot_goal_thr_;
  private:
    
  };
}

#endif
