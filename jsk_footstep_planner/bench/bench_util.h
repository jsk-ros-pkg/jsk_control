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

#include "jsk_footstep_planner/footstep_graph.h"
#include "jsk_footstep_planner/astar_solver.h"
#include "jsk_footstep_planner/footstep_astar_solver.h"

using namespace jsk_footstep_planner;
const Eigen::Vector3f resolution(0.05, 0.05, 0.08);
inline Eigen::Affine3f affineFromXYYaw(double x, double y, double yaw)
{
  return Eigen::Translation3f(x, y, 0) * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
}

inline void plan(double x, double y, double yaw,
          FootstepGraph::Ptr graph,
          Eigen::Vector3f footstep_size)
{
  Eigen::Affine3f goal_center = affineFromXYYaw(x, y, yaw);
  FootstepState::Ptr left_goal(new FootstepState(jsk_footstep_msgs::Footstep::LEFT,
                                                 goal_center * Eigen::Translation3f(0, 0.1, 0),
                                                 footstep_size,
                                                 resolution));
  FootstepState::Ptr right_goal(new FootstepState(jsk_footstep_msgs::Footstep::RIGHT,
                                                  goal_center * Eigen::Translation3f(0, -0.1, 0),
                                                  footstep_size,
                                                  resolution));
  graph->setGoalState(left_goal, right_goal);
  //AStarSolver<FootstepGraph> solver(graph);
  FootstepAStarSolver<FootstepGraph> solver(graph, 100, 100, 100);
  solver.setHeuristic(&footstepHeuristicStepCost);
  std::vector<SolverNode<FootstepState, FootstepGraph>::Ptr> path = solver.solve();
}
