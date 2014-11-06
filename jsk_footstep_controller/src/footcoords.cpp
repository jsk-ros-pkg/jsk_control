// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include <jsk_topic_tools/rosparam_utils.h>
#include "jsk_footstep_controller/footcoords.h"
#include <std_msgs/String.h>
#include <tf_conversions/tf_eigen.h>

namespace jsk_footstep_controller
{

  Footcoords::Footcoords()
  {
    ros::NodeHandle nh, pnh("~");
    tf_listener_.reset(new tf::TransformListener());
    ground_transform_.setRotation(tf::Quaternion(0, 0, 0, 1));
    // read parameter
    ground_offset_.resize(3);
    jsk_topic_tools::readVectorParameter(pnh, "ground_offset", ground_offset_);
    pnh.param("output_frame_id", output_frame_id_, std::string("odom_on_ground"));
    pnh.param("parent_frame_id", parent_frame_id_, std::string("odom"));
    pnh.param("lfoot_frame_id", lfoot_frame_id_, std::string("LLEG_LINK5"));
    pnh.param("rfoot_frame_id", rfoot_frame_id_, std::string("RLEG_LINK5"));
    pnh.param("force_threshold", force_thr_, 1.0);
    pub_state_ = pnh.advertise<std_msgs::String>("state", 1);
    before_on_the_air_ = true;
    sub_lfoot_force_.subscribe(nh, "lfsensor", 1);
    sub_rfoot_force_.subscribe(nh, "rfsensor", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_lfoot_force_, sub_rfoot_force_);
    sync_->registerCallback(boost::bind(&Footcoords::filter, this, _1, _2));
  }

  Footcoords::~Footcoords()
  {

  }



  void Footcoords::filter(const geometry_msgs::WrenchStamped::ConstPtr& lfoot,
                          const geometry_msgs::WrenchStamped::ConstPtr& rfoot)
  {
    if (lfoot->wrench.force.z < force_thr_ &&
        rfoot->wrench.force.z < force_thr_) {
      before_on_the_air_ = true;
      publishState("air");
    }
    else {
      if (lfoot->wrench.force.z > force_thr_ &&
          rfoot->wrench.force.z > force_thr_) {
        // on ground
        publishState("ground");
        if (before_on_the_air_) {
          if(updateGroundTF(lfoot->header.stamp)) {
            before_on_the_air_ = false;
          }
        }
      }
      else if (lfoot->wrench.force.z > force_thr_) {
        // only left
        publishState("lfoot");
      }
      else if (rfoot->wrench.force.z > force_thr_) {
        // only right
        publishState("rfoot");
      }
    }
    publishGroundTF(lfoot->header.stamp);
  }
  void Footcoords::publishState(const std::string& state)
  {
    std_msgs::String state_msg;
    state_msg.data = state;
    pub_state_.publish(state_msg);
  }

  bool Footcoords::updateGroundTF(const ros::Time& stamp)
  {
    // lookup tf between lfoot_frame_id -> rfoot_frame_id and 
    // odom -> lfoot_frame_id
    if (!tf_listener_->waitForTransform(
          parent_frame_id_, lfoot_frame_id_, stamp, ros::Duration(1.0))) {
      ROS_ERROR("failed to lookup transform between %s and %s",
                parent_frame_id_.c_str(),
                lfoot_frame_id_.c_str());
      return false;
    }
    else if (!tf_listener_->waitForTransform(
               parent_frame_id_, rfoot_frame_id_, stamp, ros::Duration(1.0))) {
      ROS_ERROR("failed to lookup transform between %s and %s",
                parent_frame_id_.c_str(),
                rfoot_frame_id_.c_str());
      return false;
    }
    else {                 
      try 
      {
        tf::StampedTransform lfoot_transform;
        tf_listener_->lookupTransform(
          parent_frame_id_, lfoot_frame_id_, stamp, lfoot_transform);
        tf::StampedTransform rfoot_transform;
        tf_listener_->lookupTransform(
          parent_frame_id_, rfoot_frame_id_, stamp, rfoot_transform);
        tf::Quaternion lfoot_rot = lfoot_transform.getRotation();
        tf::Quaternion rfoot_rot = rfoot_transform.getRotation();
        tf::Quaternion mid_rot = lfoot_rot.slerp(rfoot_rot, 0.5);
        tf::Vector3 lfoot_pos = lfoot_transform.getOrigin();
        tf::Vector3 rfoot_pos = rfoot_transform.getOrigin();
        tf::Vector3 mid_pos = lfoot_pos.lerp(rfoot_pos, 0.5);
        ground_transform_.setOrigin(
          mid_pos + tf::Vector3(ground_offset_[0],
                                ground_offset_[1],
                                ground_offset_[2]));
        ground_transform_.setRotation(mid_rot);
        return true;
      }
      catch (tf2::ConnectivityException &e)
      {
        ROS_ERROR("transform error: %s", e.what());
        return false;
      }
      catch (tf2::InvalidArgumentException &e)
      {
        ROS_ERROR("transform error: %s", e.what());
        return false;
      }
            
    }
  }

  void Footcoords::publishGroundTF(const ros::Time& stamp)
  {
    tf_broadcaster_.sendTransform(tf::StampedTransform(ground_transform_, 
                                                       stamp,
                                                       parent_frame_id_,
                                                       output_frame_id_
                                    ));
                                          
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "footcoords");
  jsk_footstep_controller::Footcoords c;
  ros::spin();
}
