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

#include "jsk_footstep_planner/pointcloud_model_generator.h"

namespace jsk_footstep_planner
{
  void PointCloudModelGenerator::generate(
    const std::string& model_name,
    pcl::PointCloud<PointT>& output)
  {
    output.points.clear();
    if (model_name == "flat") {
      flat(output);
    }
    else if (model_name == "stairs") {
      stairs(output);
    }
    else if (model_name == "hills") {
      hills(output);
    }
  }

  void PointCloudModelGenerator::flat(pcl::PointCloud<PointT>& output)
  {
    for (double y = -2; y < 2; y = y + 0.01) {
      for (double x = -2; x < 2; x = x + 0.01) {
        pcl::PointNormal p;
        p.x = x;
        p.y = y;
        output.points.push_back(p);
      }
    }
  }

  void PointCloudModelGenerator::hills(pcl::PointCloud<PointT>& output)
  {
    const double height = 0.1;
    for (double y = -2; y < 2; y = y + 0.01) {
      for (double x = -2; x < 2; x = x + 0.01) {
        pcl::PointNormal p;
        p.x = x;
        p.y = y;
        p.z = height * sin(x * 2) * sin(y * 2);
        output.points.push_back(p);
      }
    }
  }

  void PointCloudModelGenerator::stairs(pcl::PointCloud<PointT>& output)
  {
    for (double y = -2; y < 2; y = y + 0.01) {
      for (double x = -1; x < 0; x = x + 0.01) {
        pcl::PointNormal p;
        p.x = x;
        p.y = y;
        p.z = 0;
        output.points.push_back(p);
      }
      for (double x = 0; x < 5; x = x + 0.01) {
        pcl::PointNormal p;
        p.x = x;
        p.y = y;
        p.z = floor(x * 3) * 0.1;
        output.points.push_back(p);
      }
    }
  }
  
}
