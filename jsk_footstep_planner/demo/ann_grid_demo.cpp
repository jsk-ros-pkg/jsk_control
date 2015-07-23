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
#include "jsk_footstep_planner/ann_grid.h"

using namespace jsk_footstep_planner;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ann_grid_demo");
  pcl::PointCloud<pcl::PointNormal> cloud;
  for (double x = -2; x < 2; x += 0.01) {
    for (double y = -2; y < 2; y += 0.01) {
      pcl::PointNormal p;
      p.x = x;
      p.y = y;
      p.z = cos(x) * cos(y);
      cloud.points.push_back(p);
    }
  }
  ANNGrid::Ptr ann_grid(new ANNGrid(0.1));
  ann_grid->build(cloud);
  cv::Mat grid_image;
  ann_grid->toImage(grid_image);
  std::cout << "ann grid size is "
            << grid_image.rows << "x" << grid_image.cols << std::endl;
  cv::imshow("ann_grid", grid_image);

  cv::Mat grid_image2;
  ANNGrid::IndexArray pixels = ann_grid->bresenham(Eigen::Vector3f(1, 1, 0), Eigen::Vector3f(-1, -1, 0));
  ann_grid->toImage(grid_image2, pixels);
  cv::imshow("ann_grid2", grid_image2);

  cv::Mat grid_image3;
  ANNGrid::IndexArray box_pixels = ann_grid->box(Eigen::Vector3f(1, 1, 0),
                                                 Eigen::Vector3f(-1, 1, 0),
                                                 Eigen::Vector3f(-1, -1, 0),
                                                 Eigen::Vector3f(1, -1, 0));
  ann_grid->toImage(grid_image3, box_pixels);
  cv::imshow("ann_grid3", grid_image3);

  cv::Mat grid_image4;
  ANNGrid::IndexArray filled_pixels = ann_grid->fillByBox(Eigen::Vector3f(1, 1, 0),
                                                          Eigen::Vector3f(-1, 1, 0),
                                                          Eigen::Vector3f(-1, -1, 0),
                                                          Eigen::Vector3f(1, -1, 0));
  std::cout << "filled pixels: " << filled_pixels.size() << std::endl;
  ann_grid->toImage(grid_image4, filled_pixels);
  cv::imshow("ann_grid4", grid_image4);

  pcl::PointIndices near_indices;
  ann_grid->approximateSearchInBox(Eigen::Vector3f(1, 1, 0),
                                   Eigen::Vector3f(-1, 1, 0),
                                   Eigen::Vector3f(-1, -1, 0),
                                   Eigen::Vector3f(1, -1, 0),
                                   near_indices);
  for (size_t i = 0; i < near_indices.indices.size(); i++) {
    std::cout << cloud.points[near_indices.indices[i]] << std::endl;
  }
  
  while(1) {
    cv::waitKey(0);
  }
  return 0;
}
