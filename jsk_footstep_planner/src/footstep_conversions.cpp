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

#include "jsk_footstep_planner/footstep_conversions.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"

namespace jsk_footstep_planner
{
  jsk_footstep_msgs::Footstep footstepFromEigenPose(Eigen::Affine3f pose)
  {
    jsk_footstep_msgs::Footstep footstep;
    tf::poseEigenToMsg(pose, footstep.pose);
    return footstep;
  }
  jsk_footstep_msgs::Footstep footstepFromEigenPose(Eigen::Affine3d pose)
  {
    jsk_footstep_msgs::Footstep footstep;
    tf::poseEigenToMsg(pose, footstep.pose);
    return footstep;
  }
  
  visualization_msgs::Marker footstepToMarker(const jsk_footstep_msgs::Footstep& footstep,
                                              const std_msgs::Header& header)
  {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale = footstep.dimensions;
    marker.color.a = 1.0;
    marker.pose = footstep.pose;
    if (footstep.leg == jsk_footstep_msgs::Footstep::LEFT) {
      marker.color.g = 1.0;
    }
    else {
      marker.color.r = 1.0;
    }
    return marker;
  }
  
  visualization_msgs::MarkerArray footstepArrayToMarkerArray(const jsk_footstep_msgs::FootstepArray& footstep_array)
  {
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < footstep_array.footsteps.size(); i++) {
      jsk_footstep_msgs::Footstep footstep = footstep_array.footsteps[i];
      visualization_msgs::Marker marker = footstepToMarker(footstep, footstep_array.header);
      marker_array.markers.push_back(marker);
    }
    return marker_array;
  }
}
