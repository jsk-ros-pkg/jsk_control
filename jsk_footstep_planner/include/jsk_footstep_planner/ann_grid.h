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


#ifndef JSK_FOOTSTEP_PLANNER_ANN_GRID_H_
#define JSK_FOOTSTEP_PLANNER_ANN_GRID_H_

#include <boost/tuple/tuple.hpp>
#include <boost/unordered/unordered_set.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>

namespace jsk_footstep_planner
{
  class ANNGridCell
  {
  public:
    typedef boost::shared_ptr<ANNGridCell> Ptr;
    typedef boost::unordered_set<size_t> Indices;
    ANNGridCell() {}
    virtual ~ANNGridCell() {}
    virtual void add(size_t i)
    {
      indices_.insert(i);
    }
    virtual Indices get()
    {
      return indices_;
    }

    virtual void fill(Indices& out)
    {
      for (Indices::iterator it = indices_.begin(); it != indices_.end(); ++it) {
        out.insert(*it);
      }
    }
    virtual void fill(std::vector<int>& out)
    {
      for (Indices::iterator it = indices_.begin(); it != indices_.end(); ++it) {
        out.push_back(*it);
      }
    }
  protected:
    Indices indices_;
  private:
    
  };


  /**
   * @brief
   * ANNGrid is a class to provide approximate near neighbors search based on
   * 2.5-D representation.
   * All the z values of pointcloud is ignored and it sorted as 2-D array.
   */
  class ANNGrid
  {
  public:
    typedef boost::shared_ptr<ANNGrid> Ptr;
    typedef cv::Point Index;
    typedef std::vector<Index> IndexArray;
    ANNGrid(const double grid_size): grid_size_(grid_size) {}
    virtual ~ANNGrid()
    {
    }
    
    virtual void build(const pcl::PointCloud<pcl::PointNormal>& cloud);
    
    virtual Index pointToIndex(const pcl::PointNormal& p) const
    {
      Eigen::Vector3f v = p.getVector3fMap();
      Eigen::Vector3f translated_point = v - min_point_;
      int i = (int)std::floor(translated_point[0] / grid_size_);
      int j = (int)std::floor(translated_point[1] / grid_size_);
      return cv::Point(i, j);
    }
    
    virtual Index pointToIndex(const Eigen::Vector3f& p) const
    {
      Eigen::Vector3f translated_point = p - min_point_;
      int i = (int)std::floor(translated_point[0] / grid_size_);
      int j = (int)std::floor(translated_point[1] / grid_size_);
      return cv::Point(i, j);
    }

    inline ANNGridCell::Ptr getCell(size_t i, size_t j)
    {
      // TODO: is this check correct?
      if (cells_.size() > i) {
        if (cells_[i].size() > j) {
          return cells_[i][j];
        }
        else {
          return ANNGridCell::Ptr();
        }
      }
      else {
        return ANNGridCell::Ptr();
      }
    }
    
    virtual void approximateSearch(const Eigen::Vector3f& v, pcl::PointIndices& indices);
    virtual IndexArray bresenham(const Eigen::Vector3f& p0, const Eigen::Vector3f& p1);
    // fill method is not mt safe
    virtual IndexArray fill(const IndexArray& filled);
    virtual IndexArray fillByBox(
      const Eigen::Vector3f& p0, const Eigen::Vector3f& p1,
      const Eigen::Vector3f& p2, const Eigen::Vector3f& p3);
    virtual void approximateSearchInBox(
      const Eigen::Vector3f& p0, const Eigen::Vector3f& p1,
      const Eigen::Vector3f& p2, const Eigen::Vector3f& p3,
      pcl::PointIndices& indices);

    virtual IndexArray box(const Eigen::Vector3f& p0, const Eigen::Vector3f& p1,
                           const Eigen::Vector3f& p2, const Eigen::Vector3f& p3);
    virtual void toImage(cv::Mat& mat);
    virtual void toImage(cv::Mat& mat, const IndexArray& pixels);
  protected:

    const double grid_size_;
    std::vector<std::vector<ANNGridCell::Ptr> > cells_;
    cv::Mat mat_;
    Eigen::Vector3f min_point_;
  private:
    
  };
}

#endif
