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

#include "jsk_footstep_planner/ann_grid.h"

#include <pcl/common/common.h>
#include <jsk_topic_tools/log_utils.h>

namespace jsk_footstep_planner
{
  void ANNGrid::build(const pcl::PointCloud<pcl::PointNormal>& cloud)
  {
    cells_.clear();
    pcl::PointNormal min_pt, max_pt;
    pcl::getMinMax3D(cloud, min_pt, max_pt);
    min_point_ = min_pt.getVector3fMap();
    Eigen::Vector3f diff = max_pt.getVector3fMap() - min_pt.getVector3fMap();

    size_t x_num = std::ceil(diff[0] / grid_size_) + 1;
    size_t y_num = std::ceil(diff[1] / grid_size_) + 1;
    cells_.resize(x_num);
    mat_ = cv::Mat(y_num, x_num, CV_8U);
    for (size_t xi = 0; xi < x_num; xi++) {
      cells_[xi].resize(y_num);
      for (size_t yi = 0; yi < y_num; yi++) {
        cells_[xi][yi] = ANNGridCell::Ptr(new ANNGridCell);
      }
    }
    for (size_t i = 0; i < cloud.points.size(); i++) {
      pcl::PointNormal p = cloud.points[i];
      if (isnan(p.x) || isnan(p.y) || isnan(p.z)) {
        continue;
      }
      Index index = pointToIndex(p);
      if (index.x >= x_num) {
        ROS_FATAL("index.x exceeds x_num: %d > %lu", index.x, x_num);
      }
      if (index.y >= y_num) {
        ROS_FATAL("indey.y eyceeds y_num: %d > %lu", index.y, y_num);
      }
      cells_[index.x][index.y]->add(i);
    }
  }

  void ANNGrid::approximateSearchInBox(
    const Eigen::Vector3f& p0, const Eigen::Vector3f& p1,
    const Eigen::Vector3f& p2, const Eigen::Vector3f& p3,
    pcl::PointIndices& out_indices)
  {
    IndexArray cell_indices = fillByBox(p0, p1, p2, p3);
    //ANNGridCell::Indices point_indices;
    out_indices.indices.clear();
    for (size_t i = 0; i < cell_indices.size(); i++) {
      ANNGridCell::Ptr cell = getCell(cell_indices[i].x, cell_indices[i].y);
      if (cell) {
        cell->fill(out_indices.indices);
      }
    }
    //out_indices.indices = std::vector<int>(point_indices.begin(), point_indices.end());
  }
  
  void ANNGrid::approximateSearch(const Eigen::Vector3f& v, pcl::PointIndices& indices)
  {
    Index index = pointToIndex(v);
    ANNGridCell::Indices cell_indices = getCell(index.x, index.y)->get();
    indices.indices = std::vector<int>(cell_indices.begin(), cell_indices.end());
  }

  ANNGrid::IndexArray ANNGrid::bresenham(
    const Eigen::Vector3f& p0, const Eigen::Vector3f& p1)
  {
    IndexArray ret;
    Index i0 = pointToIndex(p0);
    Index i1 = pointToIndex(p1);
    
    int x0 = i0.x;
    int y0 = i0.y;
    int x1 = i1.x;
    int y1 = i1.y;
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx, sy;
    if (x0 < x1) {
      sx = 1;
    }
    else {
      sx = -1;
    }
    if (y0 < y1) {
      sy = 1;
    }
    else {
      sy = -1;
    }
    int err = dx - dy;
    while (1) {
      ret.push_back(cv::Point(x0, y0));
      if (x0 == x1 && y0 == y1) {
        return ret;
      }
      int e2 = 2 * err;
      if (e2 > -dy) {
        err = err - dy;
        x0 = x0 + sx;
      }
      if (e2 < dx) {
        err = err + dx;
        y0 = y0 + sy;
      }
    }
  }

  ANNGrid::IndexArray ANNGrid::fillByBox(
    const Eigen::Vector3f& p0, const Eigen::Vector3f& p1,
    const Eigen::Vector3f& p2, const Eigen::Vector3f& p3)
  {
    IndexArray box_indices(4);
    Index i0 = pointToIndex(p0);
    Index i1 = pointToIndex(p1);
    Index i2 = pointToIndex(p2);
    Index i3 = pointToIndex(p3);
    box_indices[0] = i0;
    box_indices[1] = i1;
    box_indices[2] = i2;
    box_indices[3] = i3;
    return fill(box_indices);
  }
  
  ANNGrid::IndexArray ANNGrid::fill(const IndexArray& filled)
  {
    // clear by 0
    mat_ = cv::Scalar::all(0);
    std::vector<cv::Point> pts(filled.size());
    for (size_t i = 0; i < filled.size(); i++) {
      pts[i] = cv::Point(filled[i].x, filled[i].y);
    }
    cv::fillConvexPoly(mat_, pts, cv::Scalar(255));
    // Compute bounding box
    cv::Rect bbox = cv::boundingRect(cv::Mat(pts));
    IndexArray ret;
    ret.reserve(filled.size());
    for (size_t j = 0; j <= bbox.height; j++) {
      int y = bbox.y + j;
      for (size_t i = 0; i <= bbox.width; i++) {
        int x = bbox.x + i;
        if (mat_.at<unsigned char>(y, x) == 255) {
          ret.push_back(cv::Point(x, y));
        }
      }
    }
    return ret;
  }
  
  ANNGrid::IndexArray ANNGrid::box(const Eigen::Vector3f& p0, const Eigen::Vector3f& p1,
                                   const Eigen::Vector3f& p2, const Eigen::Vector3f& p3)
  {
    IndexArray a = bresenham(p0, p1);
    IndexArray b = bresenham(p1, p2);
    IndexArray c = bresenham(p2, p3);
    IndexArray d = bresenham(p3, p0);
    IndexArray ret(a.size() + b.size() + c.size() + d.size());
    for (size_t i = 0; i < a.size(); i++) {
      ret[i] = a[i];
    }
    for (size_t i = 0; i < b.size(); i++) {
      ret[i + a.size()] = b[i];
    }
    for (size_t i = 0; i < c.size(); i++) {
      ret[i + a.size() + b.size()] = c[i];
    }
    for (size_t i = 0; i < d.size(); i++) {
      ret[i+ a.size() + b.size() + c.size()] = d[i];
    }
    return ret;
  }
  
  void ANNGrid::toImage(cv::Mat& mat)
  {
    mat = cv::Mat::zeros(cells_[0].size(), cells_.size(), CV_8U);
  }
  
  void ANNGrid::toImage(cv::Mat& mat, const IndexArray& pixels)
  {
    // Initialize by black
    toImage(mat);
    for (size_t i = 0; i < pixels.size(); i++) {
      Index id = pixels[i];
      mat.at<char>(id.y, id.x) = char(255);
    }
  }
}
