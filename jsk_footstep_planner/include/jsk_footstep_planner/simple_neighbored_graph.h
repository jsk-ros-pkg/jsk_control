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


#ifndef JSK_FOOTSTEP_PLANNER_SIMPLE_NEIGHBORED_GRAPH_
#define JSK_FOOTSTEP_PLANNER_SIMPLE_NEIGHBORED_GRAPH_

#include <string>
#include "jsk_footstep_planner/graph.h"

namespace jsk_footstep_planner
{
  class SimpleNeighboredNode
  {
  public:
    typedef boost::shared_ptr<SimpleNeighboredNode> Ptr;
    
    SimpleNeighboredNode(const std::string& name): name_(name)
    {

    }
    
    virtual void addNeighbor(SimpleNeighboredNode::Ptr node)
    {
      neighbors_.push_back(node);
    }
    
    virtual
    std::vector<SimpleNeighboredNode::Ptr>
    getNeighbors()
    {
      return neighbors_;
    }
    
    virtual std::string getName() { return name_; }
    bool operator==(SimpleNeighboredNode& other)
    {
      return name_ == other.getName();
    }
    
  protected:
    std::string name_;
    std::vector<SimpleNeighboredNode::Ptr> neighbors_;
  private:
    
  };
  
  class SimpleNeighboredGraph: public Graph<SimpleNeighboredNode>
  {
  public:
    typedef boost::shared_ptr<SimpleNeighboredGraph> Ptr;
    
    SimpleNeighboredGraph() {}
    virtual std::vector<SimpleNeighboredGraph::StatePtr> successors(
      StatePtr target_state)
    {
      return target_state->getNeighbors();
    }
    
    virtual double pathCost(StatePtr from, StatePtr to, double prev_cost)
    {
      return prev_cost + 1;
    }

    virtual bool isGoal(StatePtr state)
    {
      return *goal_state_ == *state;
    }

    virtual StatePtr
    findNode(const std::string& name)
    {
      for (size_t i = 0; i < nodes_.size(); i++) {
        StatePtr s = nodes_[i];
        if (s->getName() == name) {
          return s;
        }
      }
      return StatePtr();
    }
    
  protected:
  private:
    
  };
}

#endif
