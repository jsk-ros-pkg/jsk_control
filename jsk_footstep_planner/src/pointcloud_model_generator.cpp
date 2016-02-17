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
#include <boost/random.hpp>

namespace jsk_footstep_planner
{
  void PointCloudModelGenerator::generate(
    const std::string& model_name,
    pcl::PointCloud<PointT>& output,
    double hole_rate)
  {
    output.points.clear();
    if (model_name == "flat") {
      flat(output, hole_rate);
    }
    else if (model_name == "stairs") {
      stairs(output, hole_rate);
    }
    else if (model_name == "hills") {
      hills(output, hole_rate);
    }
    else if (model_name == "gaussian") {
      gaussian(output, hole_rate);
    }
    else if (model_name == "flat_pole") {
      flatPole(output, hole_rate);
    }
  }

  void PointCloudModelGenerator::flat(pcl::PointCloud<PointT>& output, double hole_rate)
  {
    boost::mt19937 gen( static_cast<unsigned long>(time(0)) );
    boost::uniform_real<> dst( 0, 100 );
    boost::variate_generator<
      boost::mt19937&, boost::uniform_real<>
      > rand( gen, dst );

    for (double y = -4; y < 4; y = y + 0.01) {
      for (double x = -4; x < 4; x = x + 0.01) {
        if (rand() >= hole_rate) {
          pcl::PointNormal p;
          p.x = x;
          p.y = y;
          output.points.push_back(p);
        }
      }
    }
  }

  void PointCloudModelGenerator::addPole(pcl::PointCloud<PointT>& output,
                                         const Eigen::Vector3f& center,
                                         const double width,
                                         const double height)
  {
    double y0 = center[1] + width / 2.0;
    double y1 = center[1] - width / 2.0;
    double x0 = center[0] + width / 2.0;
    double x1 = center[0] - width / 2.0;
    for (double x = x0; x > x1; x = x - 0.01) {
      for (double z = 0; z < height; z = z + 0.01) {
        pcl::PointNormal p;
        p.x = x;
        p.y = y0;
        p.z = z;
        output.points.push_back(p);
      }
    }
    for (double y = y0; y > y1; y = y - 0.01) {
      for (double z = 0; z < height; z = z + 0.01) {
        pcl::PointNormal p;
        p.x = x1;
        p.y = y;
        p.z = z;
        output.points.push_back(p);
      }
    }
    for (double x = x0; x > x1; x = x - 0.01) {
      for (double z = 0; z < height; z = z + 0.01) {
        pcl::PointNormal p;
        p.x = x;
        p.y = y1;
        p.z = z;
        output.points.push_back(p);
      }
    }
    for (double y = y0; y > y1; y = y - 0.01) {
      for (double z = 0; z < height; z = z + 0.01) {
        pcl::PointNormal p;
        p.x = x0;
        p.y = y;
        p.z = z;
        output.points.push_back(p);
      }
    }
  }
  
  void PointCloudModelGenerator::flatPole(pcl::PointCloud<PointT>& output, double hole_rate)
  {
    boost::mt19937 gen( static_cast<unsigned long>(time(0)) );
    boost::uniform_real<> dst( 0, 100 );
    boost::variate_generator<
      boost::mt19937&, boost::uniform_real<>
      > rand( gen, dst );

    for (double y = -4; y < 4; y = y + 0.01) {
      for (double x = -4; x < 4; x = x + 0.01) {
        if (rand() >= hole_rate) {
          pcl::PointNormal p;
          p.x = x;
          p.y = y;
          output.points.push_back(p);
        }
      }
    }
    for (double y = -4; y < 4; y = y + 2.0) {
      for (double x = -4; x < 4; x = x + 2.0) {
        if (x != 0.0 || y != 0.0) {
          addPole(output, Eigen::Vector3f(x, y, 0), 0.2, 2.0);
        }
      }
    }
  }

  void PointCloudModelGenerator::hills(pcl::PointCloud<PointT>& output, double hole_rate)
  {
    boost::mt19937 gen( static_cast<unsigned long>(time(0)) );
    boost::uniform_real<> dst( 0, 100 );
    boost::variate_generator<
      boost::mt19937&, boost::uniform_real<>
      > rand( gen, dst );

    const double height = 0.1;
    for (double y = -4; y < 4; y = y + 0.01) {
      for (double x = -4; x < 4; x = x + 0.01) {
        if (rand() >= hole_rate) {
          pcl::PointNormal p;
          p.x = x;
          p.y = y;
          p.z = height * sin(x * 2) * sin(y * 2);
          output.points.push_back(p);
        }
      }
    }
  }

  void PointCloudModelGenerator::gaussian(pcl::PointCloud<PointT>& output, double hole_rate)
  {
    boost::mt19937 gen( static_cast<unsigned long>(time(0)) );
    boost::uniform_real<> dst( 0, 100 );
    boost::variate_generator<
      boost::mt19937&, boost::uniform_real<>
      > rand( gen, dst );
    const double height = 1.0;
    const double sigma = 0.3;
    for (double y = -4; y < 4; y = y + 0.01) {
      for (double x = -4; x < 4; x = x + 0.01) {
        if (rand() >= hole_rate) {
          pcl::PointNormal p;
          p.x = x;
          p.y = y;
          //p.z = height * sin(x * 2) * sin(y * 2);
          p.z = height * exp(-x*x / (2 * sigma * 2)) * exp(-y*y / (2 * sigma * 2));
          output.points.push_back(p);
        }
      }
    }
  }

  void PointCloudModelGenerator::stairs(pcl::PointCloud<PointT>& output, double hole_rate)
  {
    boost::mt19937 gen( static_cast<unsigned long>(time(0)) );
    boost::uniform_real<> dst( 0, 100 );
    boost::variate_generator<
      boost::mt19937&, boost::uniform_real<>
      > rand( gen, dst );

    for (double y = -4; y < 4; y = y + 0.01) {
      // for (double x = -4; x < 0; x = x + 0.01) {
      //   if (rand() >= hole_rate) {
      //     pcl::PointNormal p;
      //     p.x = x;
      //     p.y = y;
      //     p.z = 0;
      //     output.points.push_back(p);
      //   }
      // }
      for (double x = -4; x < 5; x = x + 0.01) {
        if (rand() >= hole_rate) {
          pcl::PointNormal p;
          p.x = x;
          p.y = y;
          if (x > 0) {
            p.z = floor(x * 3) * 0.1;
          }
          else {
            p.z = ceil(x * 3) * 0.1;
          }
          output.points.push_back(p);
        }
      }
    }
  }
  
}
