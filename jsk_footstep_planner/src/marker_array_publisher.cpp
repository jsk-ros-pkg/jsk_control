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

#include "jsk_footstep_planner/marker_array_publisher.h"

namespace jsk_footstep_planner
{
  MarkerArrayPublisher::MarkerArrayPublisher(ros::NodeHandle& nh, const std::string& topic)
  {
    pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic, 1, true);
  }

  void MarkerArrayPublisher::insert(const std::string& name, visualization_msgs::Marker marker)
  {
    marker.id = getID(name);
    markers_[name] = marker;
  }

  void MarkerArrayPublisher::publish()
  {
    visualization_msgs::MarkerArray marker_array;
    for (std::map<std::string, visualization_msgs::Marker>::iterator it = markers_.begin();
         it != markers_.end();
         ++it) {
      marker_array.markers.push_back(it->second);
    }
    pub_.publish(marker_array);
  }

  void MarkerArrayPublisher::clear(const std::string& name)
  {
    if (markers_.count(name) == 0) {
      // do nothing
    }
    else {
      visualization_msgs::Marker clear_marker;
      clear_marker.id = getID(name);
      clear_marker.action = visualization_msgs::Marker::DELETE;
      markers_[name] = clear_marker;
    }
  }
  
  void MarkerArrayPublisher::clear()
  {
    for (std::map<std::string, visualization_msgs::Marker>::iterator it = markers_.begin();
         it != markers_.end();
         ++it) {
      clear(it->first);
    }
  }
  
  size_t MarkerArrayPublisher::getID(const std::string& name)
  {
    if (name_mapping_.count(name) == 0) {
      name_mapping_[name] = name_mapping_.size();
    }
    else {
      return name_mapping_[name];
    }
  }
}
