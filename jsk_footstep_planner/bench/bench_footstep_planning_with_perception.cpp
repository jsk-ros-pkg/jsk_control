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
#include "bench_util.h"
#include <time.h>
#include <boost/random.hpp>
#include <fstream>

using namespace jsk_footstep_planner;

const Eigen::Vector3f footstep_size(0.2, 0.1, 0.000001);
pcl::PointCloud<pcl::PointNormal>::Ptr
generateCloudFlat()
{
  pcl::PointCloud<pcl::PointNormal>::Ptr gen_cloud(new pcl::PointCloud<pcl::PointNormal>);
  for (double y = -5; y < 5; y = y + 0.01) {
    for (double x = -5; x < 5; x = x + 0.01) {
      pcl::PointNormal p;
      p.x = x;
      p.y = y;
      gen_cloud->points.push_back(p);
    }
  }
  return gen_cloud;
}



void setupGraph(FootstepGraph::Ptr graph)
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
                                             resolution));
  graph->setStartState(start);
  graph->setBasicSuccessors(successors);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bench_footstep_planning_with_perception");
  ros::NodeHandle nh("~");
  //graph->setProgressPublisher(nh, "progress");
  // set successors
  const size_t trials = 10;
  const double dx = 0.2;
  for (size_t ti = 0; ti < 8; ti++) {
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud = generateCloudFlat();
    FootstepGraph::Ptr graph(new FootstepGraph(resolution, true, true));
    graph->setPointCloudModel(cloud);
    setupGraph(graph);
    double theta = 2.0 * M_PI / 8 * ti ;
    std::ofstream ofs((boost::format("footstep_planning_with_perception-%f.csv") % theta).str().c_str());
    for (double x = -3; x <= 3.0; x += dx) {
      for (double y = -3; y <= 3.0; y += dx) {
        std::cout << x << ", " << y << ", " << theta << std::endl;
        ros::WallTime start = ros::WallTime::now();
        for (size_t i = 0; i < trials; i++) {
          plan(x, y, theta, graph, footstep_size);
        }
        ros::WallTime end = ros::WallTime::now();
        double time_to_solev = (end - start).toSec() / trials;
        ofs << (boost::format("%f,%f,%f,%f") % x % y % theta %  time_to_solev).str() << std::endl;
      }
      ofs << std::endl;
    }
  }
  std::cout << "Plot result by scripts/plot_bench.py" << std::endl;
  return 0;
}
