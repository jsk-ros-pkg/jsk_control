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


#ifndef JSK_FOOTSTEP_PLANNER_ASTAR_SOLVER_H_
#define JSK_FOOTSTEP_PLANNER_ASTAR_SOLVER_H_

#include "jsk_footstep_planner/best_first_search_solver.h"
#include <boost/function.hpp>
namespace jsk_footstep_planner
{
  template <class GraphT>
  class AStarSolver: public BestFirstSearchSolver<GraphT>
  {
  public:
    typedef boost::shared_ptr<AStarSolver> Ptr;
    typedef typename GraphT::StateT::Ptr StatePtr;
    typedef typename GraphT::StateT State;
    typedef typename GraphT::Ptr GraphPtr;
    typedef typename SolverNode<State, GraphT>::Ptr SolverNodePtr;
    typedef typename boost::function<double(SolverNodePtr, GraphPtr)> HeuristicFunction;
    
    AStarSolver(GraphPtr graph): BestFirstSearchSolver<GraphT>(graph) {}
    virtual double fn(SolverNodePtr n)
    {
      return gn(n) + hn(n);
    }

    virtual double gn(SolverNodePtr n)
    {
      return n->getCost();
    }

    virtual double hn(SolverNodePtr n)
    {
      return heuristic_(n, graph_);
    }

    virtual void setHeuristic(HeuristicFunction h) { heuristic_ = h; }
    
  protected:
    using Solver<GraphT>::graph_;
    HeuristicFunction heuristic_;
    
  private:
    
  };
}

#endif
