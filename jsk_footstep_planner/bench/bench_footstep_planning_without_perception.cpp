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
#include <jsk_footstep_msgs/FootstepArray.h>
#include "jsk_footstep_planner/footstep_graph.h"
#include "jsk_footstep_planner/astar_solver.h"
#include "jsk_footstep_planner/footstep_astar_solver.h"
#include <time.h>
#include <boost/random.hpp>
#include <fstream>
using namespace jsk_footstep_planner;

const Eigen::Vector3f footstep_size(0.2, 0.1, 0.000001);
const Eigen::Vector3f resolution(0.05, 0.05, 0.08);

Eigen::Affine3f affineFromXYYaw(double x, double y, double yaw)
{
  return Eigen::Translation3f(x, y, 0) * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
}

void plan(double x, double y, double yaw,
          FootstepGraph::Ptr graph, ros::Publisher& pub_path,
          ros::Publisher& pub_goal,
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "foootstep_planning_2d");
  ros::NodeHandle nh("~");
  ros::Publisher pub_start = nh.advertise<jsk_footstep_msgs::FootstepArray>("start", 1, true);
  ros::Publisher pub_goal = nh.advertise<jsk_footstep_msgs::FootstepArray>("goal", 1, true);
  ros::Publisher pub_path = nh.advertise<jsk_footstep_msgs::FootstepArray>("path", 1, true);
  boost::mt19937 rng( static_cast<unsigned long>(time(0)) );
  boost::uniform_real<> xyrange(-3.0,3.0);
  boost::variate_generator< boost::mt19937, boost::uniform_real<> > pos_rand(rng, xyrange);
  boost::uniform_real<> trange(0, 2 * M_PI);
  boost::variate_generator< boost::mt19937, boost::uniform_real<> > rot_rand(rng, trange);

  FootstepGraph::Ptr graph(new FootstepGraph(resolution));
  //graph->setProgressPublisher(nh, "progress");
  // set successors
  std::vector<Eigen::Affine3f> successors;
  successors.push_back(affineFromXYYaw(0, -0.2, 0));
  successors.push_back(affineFromXYYaw(0, -0.3, 0));
  successors.push_back(affineFromXYYaw(0, -0.15, 0));
  successors.push_back(affineFromXYYaw(0.2, -0.2, 0));
  successors.push_back(affineFromXYYaw(0.25, -0.2, 0));
  successors.push_back(affineFromXYYaw(0.1, -0.2, 0));
  successors.push_back(affineFromXYYaw(-0.1, -0.2, 0));
  successors.push_back(affineFromXYYaw(0, -0.2, 0.17));
  successors.push_back(affineFromXYYaw(0, -0.3, 0.17));
  successors.push_back(affineFromXYYaw(0.2, -0.2, 0.17));
  successors.push_back(affineFromXYYaw(0.25, -0.2, 0.17));
  successors.push_back(affineFromXYYaw(0.1, -0.2, 0.17));
  successors.push_back(affineFromXYYaw(0, -0.2, -0.17));
  successors.push_back(affineFromXYYaw(0, -0.3, -0.17));
  successors.push_back(affineFromXYYaw(0.2, -0.2, -0.17));
  successors.push_back(affineFromXYYaw(0.25, -0.2, -0.17));
  successors.push_back(affineFromXYYaw(0.1, -0.2, -0.17));
  FootstepState::Ptr start(new FootstepState(jsk_footstep_msgs::Footstep::LEFT,
                                             Eigen::Affine3f::Identity(),
                                             footstep_size,
                                             resolution));
  graph->setStartState(start);
  graph->setBasicSuccessors(successors);

  
  const size_t trials = 10;
  for (size_t ti = 0; ti < 8; ti++) {
    double theta = 2.0 * M_PI / 8 * ti ;
  //for (double theta = 0; theta <= 2.0 * M_PI; theta += M_PI / 4.0) {
    std::ofstream ofs((boost::format("footstep_planning_without_perception-%f.csv") % theta).str().c_str());
    for (double x = -3; x <= 3.0; x += 0.1) {
      for (double y = -3; y <= 3.0; y += 0.1) {
        std::cout << x << ", " << y << ", " << theta << std::endl;
        ros::WallTime start = ros::WallTime::now();
        for (size_t i = 0; i < trials; i++) {
          plan(x, y, theta, graph, pub_path, pub_goal, footstep_size);
        }
        ros::WallTime end = ros::WallTime::now();
        double time_to_solev = (end - start).toSec() / trials;
        ofs << (boost::format("%f,%f,%f,%f") % x % y % theta %  time_to_solev).str() << std::endl;
      }
      ofs << std::endl;
    }
  }
  std::cout << "Plot result by gnuplot:" << std::endl;
  std::cout << "  set pm3d map" << std::endl;
  std::cout << "  splot output.csv u 2:1:4" << std::endl;
  return 0;
}
