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


#ifndef JSK_FOOTSTEP_PLANNER_SOLVER_NODE_H_
#define JSK_FOOTSTEP_PLANNER_SOLVER_NODE_H_

#include <algorithm> 
#include <iterator>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "jsk_footstep_planner/graph.h"
#include <ros/ros.h>

namespace jsk_footstep_planner
{
  template <class StateT, class GraphT>
  class SolverNode
  {
  public:
    typedef boost::shared_ptr<SolverNode> Ptr;
    typedef boost::shared_ptr<StateT> StatePtr;
    typedef typename GraphT::Ptr GraphPtr;
    typedef typename boost::weak_ptr<GraphT> GraphWeakPtr;
    //typedef boost::unordered_map< StatePtr, Ptr > SolverList;

    SolverNode(StatePtr state, const double cost,
               Ptr parent, GraphPtr graph):
      cost_(cost), state_(state), parent_(parent), graph_(graph) {}

    SolverNode(StatePtr state, const double cost, GraphPtr graph):
      cost_(cost), state_(state), graph_(graph) {}

    virtual
    StatePtr getState() const { return state_; }

    virtual
    std::vector<Ptr> wrapWithSolverNodes(Ptr this_ptr, std::vector<StatePtr> successors)
    {
      GraphPtr graph_ptr = graph_.lock();
      std::vector<Ptr> solver_nodes;
      for (size_t i = 0; i < successors.size(); i++) {
        StatePtr next_state = successors[i];
        SolverNode::Ptr solver_node(new SolverNode(
                                      next_state,
                                      graph_ptr->pathCost(state_, next_state, cost_),
                                      this_ptr,
                                      graph_ptr));
        solver_nodes.push_back(solver_node);
      }
      return solver_nodes;
    }

    virtual
    std::vector<Ptr> expand(Ptr this_ptr, bool verbose)
    {
      GraphPtr graph_ptr = graph_.lock();
      std::vector<Ptr> solver_nodes;
      if (graph_ptr) {
        std::vector<StatePtr> successors = graph_ptr->successors(state_);
        if (verbose) {
          std::cerr << successors.size() << " successors" << std::endl;
        }
        return wrapWithSolverNodes(this_ptr, successors);
      }
      else {
        // TODO: should raise exception
        //ROS_FATAL("no graph is set");
        throw std::runtime_error("no graph is set in SolverNode");
      }
      return solver_nodes;
    }

    bool isRoot() const { return !parent_; }
    double getCost() const { return cost_; }
    double getSortValue() const { return sort_value_; }
    void setGraph(GraphPtr graph) { graph_ = graph; }
    void setSortValue(double v) { sort_value_ = v; }
    void setCost(double c) { cost_ = c; }

    std::vector<SolverNode::Ptr>
    getPathWithoutThis()
    {
      if (isRoot()) {
        return std::vector<SolverNode::Ptr>();
      }
      else {
        std::vector<SolverNode::Ptr> parent_path = parent_->getPathWithoutThis();
        parent_path.push_back(parent_); // recursive?
        return parent_path;
      }
    }

    virtual void setState(StatePtr state) { state_ = state; }

    friend bool operator<(const SolverNode<StateT, GraphT>::Ptr a,
                          const SolverNode<StateT, GraphT>::Ptr b)
    {
      return a->getSortValue() < b->getSortValue();
    }

    friend bool operator>(const SolverNode<StateT, GraphT>::Ptr a,
                          const SolverNode<StateT, GraphT>::Ptr b)
    {
      return a->getSortValue() > b->getSortValue();
    }

    virtual Ptr getParent() const { return parent_; }

  protected:
    double cost_;
    double sort_value_;     // for best first search
    StatePtr state_;
    Ptr parent_;
    GraphWeakPtr graph_;
    //std::vector<SolverNode::Ptr> memoized_path_;
  private:
  };

#if 0 // Not using, but for testing
  template <class StateT, class GraphT>
  class SolverNodeSingleton : public SolverNode< StateT, GraphT >
  {
  public:
    typedef boost::shared_ptr<SolverNodeSingleton> SPtr;
    typedef boost::shared_ptr< SolverNode< StateT, GraphT > > Ptr;
    typedef boost::shared_ptr<StateT> StatePtr;
    typedef typename GraphT::Ptr GraphPtr;
    typedef typename boost::weak_ptr<GraphT> GraphWeakPtr;

    using SolverNode< StateT, GraphT >::isRoot;

    SolverNodeSingleton(StatePtr state, const double cost,
                        Ptr parent, GraphPtr graph) :
      SolverNode< StateT, GraphT > (state, cost, parent, graph) {}

    SolverNodeSingleton(StatePtr state, const double cost, GraphPtr graph) :
      SolverNode< StateT, GraphT >(state, cost, graph) {}

    virtual
    std::vector<Ptr> wrapWithSolverNodes(Ptr this_ptr, std::vector<StatePtr> successors)
    {
      GraphPtr graph_ptr = graph_.lock();
      std::vector<Ptr> solver_nodes;
      for (size_t i = 0; i < successors.size(); i++) {
        StatePtr next_state = successors[i];
        Ptr solver_node(new SolverNodeSingleton(
                                next_state,
                                graph_ptr->pathCost(state_, next_state, cost_),
                                this_ptr,
                                graph_ptr));
        solver_nodes.push_back(solver_node);
      }
      return solver_nodes;
    }

    std::vector<Ptr>
    getPathWithoutThis()
    {
      if (isRoot()) {
        return std::vector<Ptr>();
      }
      else {
        std::vector<Ptr> parent_path = parent_->getPathWithoutThis();
        parent_path.push_back(graph_->getNode(parent_)); // recursive?
        return parent_path;
      }
    }
  protected:
    using SolverNode< StateT, GraphT >::cost_;
    using SolverNode< StateT, GraphT >::sort_value_;     // for best first search
    using SolverNode< StateT, GraphT >::state_;
    using SolverNode< StateT, GraphT >::parent_;
    using SolverNode< StateT, GraphT >::graph_;
  private:
  };
#endif
}

#endif
