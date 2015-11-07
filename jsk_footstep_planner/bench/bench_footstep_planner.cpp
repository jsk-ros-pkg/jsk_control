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
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include "jsk_footstep_planner/pointcloud_model_generator.h"

using namespace jsk_footstep_planner;

const Eigen::Vector3f footstep_size(0.2, 0.1, 0.000001);

#define OPTION_DEFAULT_VALUE(TYPE, VALUE) boost::program_options::value<TYPE>()->default_value(VALUE)
#define OPTION_TYPE(TYPE) boost::program_options::value<TYPE>()

void setupGraph(FootstepGraph::Ptr graph, Eigen::Vector3f res)
{
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
                                             res));
  graph->setStartState(start);
  graph->setBasicSuccessors(successors);
}

inline bool planBench(double x, double y, double yaw,
                      FootstepGraph::Ptr graph,
                      Eigen::Vector3f footstep_size,
                      const std::string heuristic,
                      const double first_rotation_weight,
                      const double second_rotation_weight,
                      Eigen::Vector3f res)
{
  Eigen::Affine3f goal_center = affineFromXYYaw(x, y, yaw);
  FootstepState::Ptr left_goal(new FootstepState(jsk_footstep_msgs::Footstep::LEFT,
                                                 goal_center * Eigen::Translation3f(0, 0.1, 0),
                                                 footstep_size,
                                                 res));
  FootstepState::Ptr right_goal(new FootstepState(jsk_footstep_msgs::Footstep::RIGHT,
                                                  goal_center * Eigen::Translation3f(0, -0.1, 0),
                                                  footstep_size,
                                                  res));
  graph->setGoalState(left_goal, right_goal);
  // Project goal and start
  if (graph->usePointCloudModel()) {
    if (!graph->projectGoal()) {
      ROS_FATAL("Failed to project goal");
      return false;
    }
  }
  //AStarSolver<FootstepGraph> solver(graph);
  FootstepAStarSolver<FootstepGraph> solver(graph, 100, 100, 100);
  if (heuristic == "step_cost") {
    solver.setHeuristic(boost::bind(&footstepHeuristicStepCost, _1, _2, first_rotation_weight, second_rotation_weight));
  }
  else if (heuristic == "zero") {
    solver.setHeuristic(&footstepHeuristicZero);
  }
  else if (heuristic == "straight") {
    solver.setHeuristic(&footstepHeuristicStraight);
  }
  else if (heuristic == "straight_rotation") {
    solver.setHeuristic(&footstepHeuristicStraightRotation);
  }
  std::vector<SolverNode<FootstepState, FootstepGraph>::Ptr> path = solver.solve(ros::WallDuration(10.0));
  return true;
}

inline void progressBar(int x, int n, int w)
{
    // Calculuate the ratio of complete-to-incomplete.
    float ratio = x/(float)n;
    int   c     = ratio * w;
 
    // Show the percentage complete.
    printf("%d/%d (%d%) [", x, n, (int)(ratio*100) );
 
    // Show the load bar.
    for (int x=0; x<c; x++)
       printf("=");
 
    for (int x=c; x<w; x++)
       printf(" ");
 
    // ANSI Control codes to go back to the
    // previous line and clear it.
    printf("]\n\033[F\033[J");
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "bench_footstep_planner", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");
  boost::program_options::options_description opt("Options");
  opt.add_options()
    ("min_x",OPTION_DEFAULT_VALUE(double, -0.3), "minimum x")
    ("max_x", OPTION_DEFAULT_VALUE(double, 3.0), "maximum x")
    ("dx", OPTION_DEFAULT_VALUE(double, 0.2), "x resolution")
    ("min_y", OPTION_DEFAULT_VALUE(double, -3.0), "minimum y")
    ("max_y", OPTION_DEFAULT_VALUE(double, 3.0), "maximum y")
    ("dy", OPTION_DEFAULT_VALUE(double, 0.2), "y resolution")
    ("resolution_x", OPTION_DEFAULT_VALUE(double, 0.05), "x resolution of local grid")
    ("resolution_y", OPTION_DEFAULT_VALUE(double, 0.05), "y resolution of local grid")
    ("resolution_theta", OPTION_DEFAULT_VALUE(double, 0.08), "theta resolution of local grid")
    ("n_theta", OPTION_DEFAULT_VALUE(int, 8), "theta resolution.")
    ("heuristic", OPTION_DEFAULT_VALUE(std::string, std::string("step_cost")),
     "heuristic function (zero, straight, straight_rotation, step_cost)")
    ("first_rotation_weight", OPTION_DEFAULT_VALUE(double, 1.0),
     "first rotation weight of step_cost heuristic")
    ("second_rotation_weight", OPTION_DEFAULT_VALUE(double, 1.0),
     "second rotation weight of step_cost heuristic")
    ("model", OPTION_DEFAULT_VALUE(std::string, std::string("none")),
     "pointcloud model (none, flat, stairs, hilld, gaussian)")
    ("enable_lazy_perception", "Enable Lazy Perception")
    ("enable_local_movement", "Enable Local Movement")
    ("test_count", OPTION_DEFAULT_VALUE(int, 10), "the number of test times to take average")
    ("output,o", OPTION_DEFAULT_VALUE(std::string, std::string("output.csv")), "output file")
    ("verbose,v", "verbose")
    ("help,h", "Show help");
  
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opt), vm);
  boost::program_options::notify(vm);

  if (vm.count("help")) {
    std::cout << opt << std::endl;
    return 0;
  }

  const double min_x = vm["min_x"].as<double>();
  const double max_x = vm["max_x"].as<double>();
  const double dx = vm["dx"].as<double>();
  const double min_y = vm["min_y"].as<double>();
  const double max_y = vm["max_y"].as<double>();
  const double dy = vm["dy"].as<double>();
  const double resolution_x = vm["resolution_x"].as<double>();
  const double resolution_y = vm["resolution_y"].as<double>();
  const double resolution_theta = vm["resolution_theta"].as<double>();
  const int n_theta = vm["n_theta"].as<int>();
  const std::string heuristic = vm["heuristic"].as<std::string>();
  const double first_rotation_weight = vm["first_rotation_weight"].as<double>();
  const double second_rotation_weight = vm["second_rotation_weight"].as<double>();
  const std::string model = vm["model"].as<std::string>();
  const bool enable_perception = vm["model"].as<std::string>() != "none";
  const bool enable_lazy_perception = vm.count("enable_lazy_perception") > 0;
  const bool enable_local_movement = vm.count("enable_local_movement") > 0;
  const bool verbose = vm.count("verbose") > 0;
  const int test_count = vm["test_count"].as<int>();
  const std::string output_filename = vm["output"].as<std::string>();

  // Print parameters
  std::cout << "parameters" << std::endl;
  std::cout << "  min_x: " << min_x << std::endl;
  std::cout << "  max_x: " << max_x << std::endl;
  std::cout << "  dx: " << dx << std::endl;
  std::cout << "  min_y: " << min_y << std::endl;
  std::cout << "  max_y: " << max_y << std::endl;
  std::cout << "  dy: " << dy << std::endl;
  std::cout << "  resolution_x: " << resolution_x << std::endl;
  std::cout << "  resolution_y: " << resolution_y << std::endl;
  std::cout << "  resolution_theta: " << resolution_theta << std::endl;
  std::cout << "  n_theta: " << n_theta << std::endl;
  std::cout << "  heuristic: " << heuristic << std::endl;
  std::cout << "  first_rotation_weight: " << first_rotation_weight << std::endl;
  std::cout << "  second_rotation_weight: " << second_rotation_weight << std::endl;
  std::cout << "  model: " << model << std::endl;
  std::cout << "  enable_lazy_perception: " << enable_lazy_perception << std::endl;
  std::cout << "  enable_local_movement: " << enable_local_movement << std::endl;
  std::cout << "  test_count: " << test_count << std::endl;
  std::cout << "  output: " << output_filename << std::endl;
  std::cout << "  verbose: " << verbose << std::endl;
  
  PointCloudModelGenerator generator;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
  if (model != "none") {
    generator.generate(model, *cloud);
  }
  Eigen::Vector3f res(resolution_x, resolution_y, resolution_theta);
  FootstepGraph::Ptr graph(new FootstepGraph(res, enable_perception, enable_lazy_perception, enable_local_movement));
  if (enable_perception) {
    graph->setPointCloudModel(cloud);
    
  }
  setupGraph(graph, res);
  if (enable_perception) {
    if (!graph->projectStart()) {
      ROS_FATAL("Failed to project start state");
      return 1;
    }
  }
  std::cout << graph->infoString() << std::endl;
  std::ofstream ofs(output_filename.c_str());
  int test_num = n_theta * ceil((max_x - min_x) / dx + 1) * ceil((max_y - min_y) / dy + 1);
  int count = 0;
  // First line, indices
  ofs << "x,y,theta,one_time,"
      << "min_x,max_x,dx,min_y,max_y,dy,"
      << "resolution_x,resolution_y,resolution_theta,n_theta,"
      << "heuristic,first_rotation_weight,second_rotation_weight,"
      << "model,"
      << "enable_lazy_perception,enable_local_movement,"
      << "test_count" << std::endl;
  
  for (size_t ti = 0; ti < n_theta; ti++) {
    double theta = 2.0 * M_PI / n_theta * ti ;
    for (double x = min_x; x <= max_x; x += dx) {
      for (double y = min_y; y <= max_y; y += dy) {
        ++count;
        progressBar(count, test_num, 80);
        ros::WallTime start = ros::WallTime::now();
        // Run plan function for test_count
        bool success = false;
        for (size_t i = 0; i < test_count; i++) {
          success = planBench(x, y, theta, graph, footstep_size, heuristic, second_rotation_weight, second_rotation_weight, res);
        }
        ros::WallTime end = ros::WallTime::now();
        if (!ros::ok()) {
          return 0;
        }
        if (verbose) {
          std::cout << "Planning took " << (end-start).toSec()/test_count << " sec" << std::endl;
        }
        if (success) {
          double time_to_solve = (end - start).toSec() / test_count;
          ofs << x << "," << y << "," << theta << "," << time_to_solve
              << "," << min_x << "," << max_x << "," << dx << "," << min_y << "," << max_y << "," << dy
              << "," << resolution_x << "," << resolution_y << "," << resolution_theta << "," << n_theta
              << "," << heuristic << "," << first_rotation_weight << "," << second_rotation_weight
              << "," << model
              << "," << enable_lazy_perception << "," << enable_local_movement
              << "," << test_count << std::endl;
        }
        else {
          ROS_FATAL("Failed to plan");
        }
        
      }
      //std::cout << std::endl;
    }
  }
  std::cout << "Plot result by scripts/plot_bench.py" << std::endl;
  return 0;
}

