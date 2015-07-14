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

#include "jsk_footstep_planner/astar_solver.h"
#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include <jsk_topic_tools/time_accumulator.h>
#include <set>

using namespace jsk_footstep_planner;

class Grid2DNode
{
public:
  typedef boost::shared_ptr<Grid2DNode> Ptr;
  Grid2DNode(int x, int y, bool occupied): x_(x), y_(y), occupied_(occupied) {}
  bool operator==(Grid2DNode& other)
  {
    return x_ == other.x_ && y_ == other.y_ ;
  }
  int getX() { return x_; }
  int getY() { return y_; }
  bool isOccupied() { return occupied_; }
protected:
  int x_;
  int y_;
  bool occupied_;
private:
  
};

class Grid2DGraph: public Graph<Grid2DNode>
{
public:
  typedef boost::shared_ptr<Grid2DGraph> Ptr;
  virtual std::vector<Grid2DGraph::StatePtr> successors(
    StatePtr target_state)
  {
    std::vector<Grid2DGraph::StatePtr> ret;
    if (target_state->getX() - 1 > 0) {
      if (state_map_[target_state->getX() - 1][target_state->getY()] &&
          !state_map_[target_state->getX() - 1][target_state->getY()]->isOccupied()) {
        ret.push_back(state_map_[target_state->getX() - 1][target_state->getY()]);
      }
    }
    if (target_state->getX() + 1 < 1024) {
      if (state_map_[target_state->getX() + 1][target_state->getY()] &&
          !state_map_[target_state->getX() + 1][target_state->getY()]->isOccupied()) {
        ret.push_back(state_map_[target_state->getX() + 1][target_state->getY()]);
      }
    }
    if (target_state->getY() - 1 > 0) {
      if (state_map_[target_state->getX()][target_state->getY() - 1] &&
          !state_map_[target_state->getX()][target_state->getY() - 1]->isOccupied()) {
        ret.push_back(state_map_[target_state->getX()][target_state->getY() - 1]);
      }
    }
    if (target_state->getY() + 1 < 1024) {
      if (state_map_[target_state->getX()][target_state->getY() + 1] &&
          !state_map_[target_state->getX()][target_state->getY() + 1]->isOccupied()) {
        ret.push_back(state_map_[target_state->getX()][target_state->getY() + 1]);
      }
    }
    return ret;
  }
  
  virtual double pathCost(StatePtr from, StatePtr to, double prev_cost)
  {
    return prev_cost + 1;
  }

  virtual bool isGoal(StatePtr state)
  {
    return goal_state_ == state;
  }
  
  virtual void addNode(StatePtr state)
  {
    Graph<Grid2DNode>::addNode(state);
    state_map_[state->getX()][state->getY()] = state;
  }

  virtual StatePtr findNode(int x, int y)
  {
    return state_map_[x][y];
  }
protected:
  StatePtr state_map_[1024][1024];
private:
  
};

double heuristicFunction(AStarSolver<Grid2DGraph>::SolverNodePtr node,
                         Grid2DGraph::Ptr graph)
{
  return 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_astar");
  Grid2DGraph::Ptr graph(new Grid2DGraph);
  // parse file
  std::string map_file = argv[1];
  std::ifstream map_stream(map_file.c_str());
  std::string input;
  size_t j = 0;
  while (std::getline(map_stream, input)) {
    for (size_t i = 0; i < input.length(); i++) {
      char c = input[i];
      Grid2DNode::Ptr node(new Grid2DNode(i, j, c == 'x'));
      graph->addNode(node);
    }
    ++j;
  }
  graph->setStartState(graph->findNode(1, 1));
  graph->setGoalState(graph->findNode(61, 28));
  {
    jsk_topic_tools::TimeAccumulator acc;
    const size_t trials = 1000;
    for (size_t i = 0; i < trials; i++) {
      jsk_topic_tools::ScopedTimer timer = acc.scopedTimer();
      AStarSolver<Grid2DGraph> solver(graph);
      solver.setHeuristic(boost::bind(&heuristicFunction, _1, _2));
      std::vector<SolverNode<Grid2DNode, Grid2DGraph>::Ptr> path
        = solver.solve();
    }
    std::cout << "A* time to solve:" << acc.mean() << std::endl;
  }
  AStarSolver<Grid2DGraph> solver(graph);
  solver.setHeuristic(boost::bind(&heuristicFunction, _1, _2));
  std::vector<SolverNode<Grid2DNode, Grid2DGraph>::Ptr> path
    = solver.solve();

  std::set<Grid2DNode::Ptr> path_states;
  for (size_t i = 0; i < path.size(); i++) {
    path_states.insert(path[i]->getState());
  }
  // print path
  for (size_t j = 0; j < 1024; j++) {
    bool is_valid_line = false;
    for (size_t i = 0; i < 1024; i++) {
      Grid2DNode::Ptr node = graph->findNode(i, j);
      if (node) {
        if (node->isOccupied()) {
          std::cout << "x";
        }
        else if (*(graph->getStartState()) == *node) {
          std::cout << "s";
        }
        else if (*(graph->getGoalState()) == *node) {
          std::cout << "g";
        }
        else if (path_states.find(node) != path_states.end()) {
          std::cout << "o";
        }
        else {
          std::cout << " ";
        }
        is_valid_line = true;
      }
    }
    if (is_valid_line) {
      std::cout << std::endl;
    }
  }
  return 0;
}
