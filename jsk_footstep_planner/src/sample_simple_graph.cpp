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
#include <ros/ros.h>

#include "jsk_footstep_planner/simple_neighbored_graph.h"
#include "jsk_footstep_planner/breadth_first_search_solver.h"
#include "jsk_footstep_planner/depth_first_search_solver.h"
#include "jsk_footstep_planner/best_first_search_solver.h"
#include "jsk_footstep_planner/astar_solver.h"
#include <iostream>
#include <boost/format.hpp>
#include <jsk_topic_tools/time_accumulator.h>

using namespace jsk_footstep_planner;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_simple_graph");
  std::cout << "usage: rosrun jsk_footstep_planner sample_astar config/map.txt" << std::endl;
  SimpleNeighboredGraph::Ptr graph (new SimpleNeighboredGraph);

  // node name: depth_num
  const int width = 10;
  const int depth = 1000;
  for (size_t d = 0; d < depth; d++) {
    for (size_t w = 0; w < width; w++) {
      SimpleNeighboredNode::Ptr a(new SimpleNeighboredNode((boost::format("%lu_%lu") % d % w).str()));
      graph->addNode(a);
      if (w != 0) {
        graph->findNode((boost::format("%lu_%lu") % d % (w - 1)).str())->addNeighbor(graph->findNode((boost::format("%lu_%lu") % d % w).str()));
        graph->findNode((boost::format("%lu_%lu") % d % w).str())->addNeighbor(graph->findNode((boost::format("%lu_%lu") % d % (w - 1)).str()));
      }
      if (d != 0) {
        graph->findNode((boost::format("%lu_%lu") % (d - 1) % w).str())->addNeighbor(graph->findNode((boost::format("%lu_%lu") % d % w).str()));
      }
    }
  }

  std::cout << "done building graph" << std::endl;
  graph->setStartState(graph->findNode("0_0"));
  graph->setGoalState(graph->findNode((boost::format("%lu_%lu") % (depth - 1) % (width - 1)).str()));
  const int trials = 10;
  {
    jsk_topic_tools::TimeAccumulator acc;
    for (size_t i = 0; i < trials; i++) {
      jsk_topic_tools::ScopedTimer timer = acc.scopedTimer();
      BreadthFirstSearchSolver<SimpleNeighboredGraph>::Ptr solver(new BreadthFirstSearchSolver<SimpleNeighboredGraph>(graph));
      std::vector<SolverNode<SimpleNeighboredNode, SimpleNeighboredGraph>::Ptr> path = solver->solve();
      std::cout << "path: " << path.size() << std::endl;
      //std::cout << i << "/" << trials << std::endl;
    }
    std::cout << "braedth first time to solve:" << acc.mean() << std::endl;
  }

  {
    jsk_topic_tools::TimeAccumulator acc;
    for (size_t i = 0; i < trials; i++) {
      jsk_topic_tools::ScopedTimer timer = acc.scopedTimer();
      DepthFirstSearchSolver<SimpleNeighboredGraph>::Ptr solver(new DepthFirstSearchSolver<SimpleNeighboredGraph>(graph));
      std::vector<SolverNode<SimpleNeighboredNode, SimpleNeighboredGraph>::Ptr> path = solver->solve();
      std::cout << "path: " << path.size() << std::endl;
      //std::cout << i << "/" << trials << std::endl;
    }
    std::cout << "depth first time to solve:" << acc.mean() << std::endl;
  }

  {
    jsk_topic_tools::TimeAccumulator acc;
    for (size_t i = 0; i < trials; i++) {
      jsk_topic_tools::ScopedTimer timer = acc.scopedTimer();
      BestFirstSearchSolver<SimpleNeighboredGraph>::Ptr solver(new BestFirstSearchSolver<SimpleNeighboredGraph>(graph));
      std::vector<SolverNode<SimpleNeighboredNode, SimpleNeighboredGraph>::Ptr> path = solver->solve();
      std::cout << "path: " << path.size() << std::endl;
      //std::cout << i << "/" << trials << std::endl;
    }
    std::cout << "best first time to solve:" << acc.mean() << std::endl;
  }
  
  BreadthFirstSearchSolver<SimpleNeighboredGraph>::Ptr solver(new BreadthFirstSearchSolver<SimpleNeighboredGraph>(graph));
  std::vector<SolverNode<SimpleNeighboredNode, SimpleNeighboredGraph>::Ptr> path = solver->solve();
  std::cout << "number of nodes: " << graph->numNodes() << std::endl;
  std::cout << "start node: " << graph->getStartState()->getName() << std::endl;
  std::cout << "goal node: " << graph->getGoalState()->getName() << std::endl;
  std::cout << "path: " << path.size() << std::endl;
  for (size_t i = 0; i < path.size(); i++) {
    std::cout << " " << i << " - " << path[i]->getState()->getName() << std::endl;
  }
  return 0;
}
