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
#include <jsk_pcl_ros/pcl_conversion_util.h>
#include <tf_conversions/tf_eigen.h>
#include <jsk_pcl_ros/pcl_conversion_util.h>

namespace jsk_footstep_controller
{

  Footcoords::Footcoords():
    diagnostic_updater_(new diagnostic_updater::Updater)
  {
    ros::NodeHandle nh, pnh("~");
    lforce_list_.resize(0);
    rforce_list_.resize(0);
    tf_listener_.reset(new tf::TransformListener());
    ground_transform_.setRotation(tf::Quaternion(0, 0, 0, 1));
    midcoords_.setRotation(tf::Quaternion(0, 0, 0, 1));
    diagnostic_updater_->setHardwareID("none");
    diagnostic_updater_->add("Support Leg Status", this, 
                             &Footcoords::updateLegDiagnostics);
    // read parameter
    pnh.param("alpha", alpha_, 0.5);
    pnh.param("sampling_time_", sampling_time_, 0.2);
    pnh.param("output_frame_id", output_frame_id_,
              std::string("odom_on_ground"));
    pnh.param("parent_frame_id", parent_frame_id_, std::string("odom"));
    pnh.param("midcoords_frame_id", midcoords_frame_id_, std::string("ground"));
    pnh.param("root_frame_id", root_frame_id_, std::string("BODY"));
    pnh.param("odom_root_frame_id", odom_root_frame_id_, std::string("odom_root"));
    pnh.param("lfoot_frame_id", lfoot_frame_id_,
              std::string("lleg_end_coords"));
    pnh.param("rfoot_frame_id", rfoot_frame_id_,
              std::string("rleg_end_coords"));
    pnh.param("lfoot_sensor_frame", lfoot_sensor_frame_, std::string("lleg_end_coords"));
    pnh.param("rfoot_sensor_frame", rfoot_sensor_frame_, std::string("rleg_end_coords"));
    // pnh.param("lfoot_sensor_frame", lfoot_sensor_frame_, std::string("lfsensor"));
    // pnh.param("rfoot_sensor_frame", rfoot_sensor_frame_, std::string("rfsensor"));
    pnh.param("force_threshold", force_thr_, 100.0);
    support_status_ = AIR;
    pub_state_ = pnh.advertise<std_msgs::String>("state", 1);
    pub_contact_state_ = pnh.advertise<jsk_footstep_controller::GroundContactState>("contact_state", 1);
    before_on_the_air_ = true;
    sub_lfoot_force_.subscribe(nh, "lfsensor", 1);
    sub_rfoot_force_.subscribe(nh, "rfsensor", 1);
    periodic_update_timer_ = pnh.createTimer(ros::Duration(1.0 / 25),
                                             boost::bind(&Footcoords::periodicTimerCallback, this, _1));
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_lfoot_force_, sub_rfoot_force_);
    sync_->registerCallback(boost::bind(&Footcoords::filter, this, _1, _2));
  }

  Footcoords::~Footcoords()
  {

  }

  void Footcoords::updateLegDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    // air -> warn
    // single leg, dual leg -> ok
    if (support_status_ == AIR) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "On Air");
    }
    else {
      if (support_status_ == LLEG_GROUND) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Left leg on the ground");
      }
      else if (support_status_ == RLEG_GROUND) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Right leg on the ground");
      }
      else if (support_status_ == BOTH_GROUND) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Both legs on the ground");
      }
    }
  }

  double Footcoords::applyLowPassFilter(double current_val, double prev_val) const
  {
    return prev_val + alpha_ * (current_val - prev_val);
  }

  bool Footcoords::allValueLargerThan(TimeStampedVector<ValueStamped::Ptr>& values,
                                      double threshold)
  {
    for (size_t i = 0; i < values.size(); i++) {
      if (values[i]->value < threshold) {
        return false;
      }
    }
    return true;
  }

  bool Footcoords::allValueSmallerThan(TimeStampedVector<ValueStamped::Ptr>& values,
                                        double threshold)
  {
    for (size_t i = 0; i < values.size(); i++) {
      if (values[i]->value > threshold) {
        return false;
      }
    }
    return true;
  }

  bool Footcoords::resolveForceTf(const geometry_msgs::WrenchStamped::ConstPtr& lfoot,
                                  const geometry_msgs::WrenchStamped::ConstPtr& rfoot,
                                  tf::Vector3& lfoot_force,
                                  tf::Vector3& rfoot_force)
  {
    try {
      if (!waitForEndEffectorTrasnformation(lfoot->header.stamp)) {
        ROS_ERROR("[Footcoords::resolveForceTf] failed to lookup transformation for sensor value");
        return false;
      }
      tf::StampedTransform lfoot_transform, rfoot_transform;
      tf_listener_->lookupTransform(
        lfoot->header.frame_id, lfoot_sensor_frame_, lfoot->header.stamp, lfoot_transform);
      tf_listener_->lookupTransform(
        rfoot->header.frame_id, rfoot_sensor_frame_, rfoot->header.stamp, rfoot_transform);
      // cancel translation
      lfoot_transform.setOrigin(tf::Vector3(0, 0, 0));
      rfoot_transform.setOrigin(tf::Vector3(0, 0, 0));

      tf::Vector3 lfoot_local, rfoot_local;
      tf::vector3MsgToTF(lfoot->wrench.force, lfoot_local);
      tf::vector3MsgToTF(rfoot->wrench.force, rfoot_local);
      lfoot_force = lfoot_transform * lfoot_local;
      rfoot_force = rfoot_transform * rfoot_local;
      // lfoot_force = lfoot_rotation * lfoot_local;
      // rfoot_force = rfoot_rotation * rfoot_local;
      return true;
    }
    catch (tf2::ConnectivityException &e) {
      ROS_ERROR("[Footcoords::resolveForceTf] transform error: %s", e.what());
      return false;
    }
    catch (tf2::InvalidArgumentException &e) {
      ROS_ERROR("[Footcoords::resolveForceTf] transform error: %s", e.what());
      return false;
    }
    catch (tf2::ExtrapolationException &e) {
      ROS_ERROR("[Footcoords::resolveForceTf] transform error: %s", e.what());
      return false;
    }
    catch (tf2::LookupException &e) {
      ROS_ERROR("[Footcoords::resolveForceTf] transform error: %s", e.what());
      return false;
    }
  }

  void Footcoords::periodicTimerCallback(const ros::TimerEvent& event)
  {
    boost::mutex::scoped_lock lock(mutex_);
    bool success_to_update = computeMidCoords(event.current_real);
    if (support_status_ == AIR || support_status_ == BOTH_GROUND) {
      publishState("ground");
    }
    else {
      if (support_status_ == LLEG_GROUND) {
        publishState("lfoot");
      }
      else if (support_status_ == RLEG_GROUND) {
        publishState("rfoot");
      }
    }
    if (success_to_update) {
      tf::StampedTransform root_transform;
      tf_listener_->lookupTransform(parent_frame_id_, root_frame_id_, event.current_real, root_transform);
      root_link_pose_.setOrigin(root_transform.getOrigin());
      root_link_pose_.setRotation(root_transform.getRotation());
      // midcoords is a center coordinates of two feet
      // compute root_link -> midcoords
      // root_link_pose_ := odom -> root_link
      // midcoords_ := odom -> midcoords
      // root_link_pose_ * T = midcoords_
      // T = root_link_pose_^-1 * midcoords_
      ground_transform_ = root_link_pose_.inverse() * midcoords_;
      publishTF(event.current_real);
      diagnostic_updater_->update();
      publishContactState(event.current_real);
    }
  }

  void Footcoords::filter(const geometry_msgs::WrenchStamped::ConstPtr& lfoot,
                          const geometry_msgs::WrenchStamped::ConstPtr& rfoot)
  {
    boost::mutex::scoped_lock lock(mutex_);
    // lowpass filter
    tf::Vector3 lfoot_force_vector, rfoot_force_vector;
    if (!resolveForceTf(lfoot, rfoot, lfoot_force_vector, rfoot_force_vector)) {
      ROS_ERROR("[Footcoords::filter] failed to resolve tf of force sensor");
      return;
    }
    // resolve tf
    double lfoot_force = applyLowPassFilter(lfoot_force_vector[2], prev_lforce_);
    double rfoot_force = applyLowPassFilter(rfoot_force_vector[2], prev_rforce_);
    lforce_list_.push_back(ValueStamped::Ptr(new ValueStamped(lfoot->header, lfoot_force)));
    rforce_list_.push_back(ValueStamped::Ptr(new ValueStamped(rfoot->header, rfoot_force)));

    prev_lforce_ = lfoot_force;
    prev_rforce_ = rfoot_force;

    if (allValueLargerThan(lforce_list_, force_thr_) &&
        allValueLargerThan(rforce_list_, force_thr_)) {
      // on ground
      if (support_status_ != BOTH_GROUND) {
        // save transformation from midcoords to odom_on_ground
        locked_midcoords_to_odom_on_ground_ = midcoords_.inverse() * ground_transform_;
      }
      support_status_ = BOTH_GROUND;
    }
    else if (allValueSmallerThan(lforce_list_, force_thr_) &&
             allValueSmallerThan(rforce_list_, force_thr_)) {
      before_on_the_air_ = true;
      support_status_ = AIR;
    }
    else if (allValueLargerThan(lforce_list_, force_thr_)) {
      // only left
      support_status_ = LLEG_GROUND;
    }
    else if (allValueLargerThan(rforce_list_, force_thr_)) {
      // only right
      support_status_ = RLEG_GROUND;
    }
    else {
      // unstable
      support_status_ = UNSTABLE;
      publishState("unstable");
    }
    lforce_list_.removeBefore(lfoot->header.stamp - ros::Duration(sampling_time_));
    rforce_list_.removeBefore(rfoot->header.stamp - ros::Duration(sampling_time_));
  }
  
  void Footcoords::publishState(const std::string& state)
  {
    std_msgs::String state_msg;
    state_msg.data = state;
    pub_state_.publish(state_msg);
  }
  
  void Footcoords::publishContactState(const ros::Time& stamp)
  {
    GroundContactState contact_state;
    contact_state.header.stamp = stamp;
    if (support_status_ == AIR) {
      contact_state.contact_state
        = GroundContactState::CONTACT_AIR;
    }
    else if (support_status_ == LLEG_GROUND) {
      contact_state.contact_state 
        = GroundContactState::CONTACT_LLEG_GROUND;
    }
    else if (support_status_ == RLEG_GROUND) {
      contact_state.contact_state 
        = GroundContactState::CONTACT_RLEG_GROUND;
    }
    else if (support_status_ == BOTH_GROUND) {
      contact_state.contact_state
        = GroundContactState::CONTACT_BOTH_GROUND;
    }
    try 
    {
      tf::StampedTransform foot_transform;
      tf_listener_->lookupTransform(
        lfoot_frame_id_, rfoot_frame_id_, stamp, foot_transform);
      double roll, pitch, yaw;
      foot_transform.getBasis().getRPY(roll, pitch, yaw);
      contact_state.error_pitch_angle = std::abs(pitch);
      contact_state.error_roll_angle = std::abs(roll);
      contact_state.error_yaw_angle = std::abs(yaw);
      contact_state.error_z = std::abs(foot_transform.getOrigin().z());
      pub_contact_state_.publish(contact_state);
    }
    catch (tf2::ConnectivityException &e)
    {
      ROS_ERROR("[Footcoords::publishContactState] transform error: %s", e.what());
    }
    catch (tf2::InvalidArgumentException &e)
    {
      ROS_ERROR("[Footcoords::publishContactState] transform error: %s", e.what());
    }
    catch (tf2::ExtrapolationException &e)
    {
      ROS_ERROR("[Footcoords::publishContactState] transform error: %s", e.what());
    }
    catch (tf2::LookupException &e)
    {
      ROS_ERROR("[Footcoords::publishContactState] transform error: %s", e.what());
    }

  }

  bool Footcoords::computeMidCoordsFromSingleLeg(const ros::Time& stamp,
                                                 bool use_left_leg)
  {
    if (!waitForEndEffectorTrasnformation(stamp)) {
      ROS_ERROR("[Footcoords::computeMidCoordsFromSingleLeg] Failed to lookup endeffector transformation");
      return false;
    }
    else {
      try
      {
        tf::StampedTransform foot_transform; // parent -> foot
        if (use_left_leg) {     // left on the ground
          tf_listener_->lookupTransform(
            parent_frame_id_, lfoot_frame_id_, stamp, foot_transform);
        }
        else {                  // right on the ground
          tf_listener_->lookupTransform(
            parent_frame_id_, rfoot_frame_id_, stamp, foot_transform);
        }
        midcoords_ = foot_transform;
        return true;
      }
      catch (tf2::ConnectivityException &e)
      {
        ROS_ERROR("[Footcoords::computeMidCoordsFromSingleLeg] transform error: %s", e.what());
        return false;
      }
      catch (tf2::InvalidArgumentException &e)
      {
        ROS_ERROR("[Footcoords::computeMidCoordsFromSingleLeg] transform error: %s", e.what());
        return false;
      }
      catch (tf2::ExtrapolationException &e)
      {
        ROS_ERROR("[Footcoords::computeMidCoordsFromSingleLeg] transform error: %s", e.what());
      }
      catch (tf2::LookupException &e)
      {
        ROS_ERROR("[Footcoords::computeMidCoordsFromSingleLeg] transform error: %s", e.what());
      }


    }
  }
  
  bool Footcoords::computeMidCoords(const ros::Time& stamp)
  {
    if (!waitForEndEffectorTrasnformation(stamp)) {
      ROS_ERROR("[Footcoords::computeMidCoords] Failed to lookup endeffector transformation");
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
        midcoords_.setOrigin(mid_pos);
        midcoords_.setRotation(mid_rot);
        return true;
      }
      catch (tf2::ConnectivityException &e)
      {
        ROS_ERROR("[Footcoords::computeMidCoords] transform error: %s", e.what());
        return false;
      }
      catch (tf2::InvalidArgumentException &e)
      {
        ROS_ERROR("[Footcoords::computeMidCoords] transform error: %s", e.what());
        return false;
      }
      catch (tf2::ExtrapolationException &e)
      {
        ROS_ERROR("[Footcoords::computeMidCoords] transform error: %s", e.what());
      }
      catch (tf2::LookupException &e)
      {
        ROS_ERROR("[Footcoords::computeMidCoords] transform error: %s", e.what());
      }

    }
  }

  bool Footcoords::waitForSensorFrameTransformation(const ros::Time& stamp,
                                                   const std::string& lsensor_frame,
                                                   const std::string& rsensor_frame)
  {
    // lfsensor -> lleg_end_coords
    if (!tf_listener_->waitForTransform(
          lsensor_frame, lfoot_sensor_frame_, stamp, ros::Duration(1.0))) {
      ROS_ERROR("[Footcoords::waitForSensorFrameTransformation] failed to lookup transform between %s and %s",
                lsensor_frame.c_str(),
                lfoot_sensor_frame_.c_str());
      return false;
    }
    if (!tf_listener_->waitForTransform(
          rsensor_frame, rfoot_sensor_frame_, stamp, ros::Duration(1.0))) {
      ROS_ERROR("[Footcoords::waitForSensorFrameTransformation] failed to lookup transform between %s and %s",
                rsensor_frame.c_str(),
                rfoot_sensor_frame_.c_str());
      return false;
    }
    return true;
  }

  bool Footcoords::waitForEndEffectorTrasnformation(const ros::Time& stamp)
  {
    // odom -> lfoot
    if (!tf_listener_->waitForTransform(
          parent_frame_id_, lfoot_frame_id_, stamp, ros::Duration(1.0))) {
      ROS_ERROR("[Footcoords::waitForEndEffectorTrasnformation] failed to lookup transform between %s and %s",
                parent_frame_id_.c_str(),
                lfoot_frame_id_.c_str());
      return false;
    }
    // odom -> rfoot
    else if (!tf_listener_->waitForTransform(
               parent_frame_id_, rfoot_frame_id_, stamp, ros::Duration(1.0))) {
      ROS_ERROR("[Footcoords::waitForEndEffectorTrasnformation]failed to lookup transform between %s and %s",
                parent_frame_id_.c_str(),
                rfoot_frame_id_.c_str());
      return false;
    }
    // lfoot -> rfoot
    else if (!tf_listener_->waitForTransform(
               lfoot_frame_id_, rfoot_frame_id_, stamp, ros::Duration(1.0))) {
      ROS_ERROR("[Footcoords::waitForEndEffectorTrasnformation]failed to lookup transform between %s and %s",
                lfoot_frame_id_.c_str(),
                rfoot_frame_id_.c_str());
      return false;
    }
    return true;
  }

  bool Footcoords::updateGroundTF()
  {
    // project `/odom` on the plane of midcoords_
    // odom -> midcoords is available as `midcoords_`
    // we need odom -> odom_on_ground
    // 1. in order to get translation,
    //    project odom point to
    if (false && support_status_ == BOTH_GROUND) {
      // use locked_midcoords_to_odom_on_ground_ during dual stance phase
      ground_transform_ = midcoords_ * locked_midcoords_to_odom_on_ground_;
    }
    else {
      Eigen::Affine3d odom_to_midcoords;
      tf::transformTFToEigen(midcoords_, odom_to_midcoords);
      Eigen::Affine3d midcoords_to_odom = odom_to_midcoords.inverse();
      Eigen::Affine3d midcoords_to_odom_on_ground = midcoords_to_odom;
      midcoords_to_odom_on_ground.translation().z() = 0.0;
      Eigen::Vector3d zaxis;
      zaxis[0] = midcoords_to_odom_on_ground(0, 2);
      zaxis[1] = midcoords_to_odom_on_ground(1, 2);
      zaxis[2] = midcoords_to_odom_on_ground(2, 2);
      Eigen::Quaterniond rot;
      rot.setFromTwoVectors(zaxis, Eigen::Vector3d(0, 0, 1));
      midcoords_to_odom_on_ground.rotate(rot);
      Eigen::Affine3d odom_to_odom_on_ground
        = odom_to_midcoords * midcoords_to_odom_on_ground;
      tf::transformEigenToTF(odom_to_odom_on_ground,
                             ground_transform_);
    }
  }

  void Footcoords::publishTF(const ros::Time& stamp)
  {
    // publish midcoords_ and ground_cooords_
    geometry_msgs::TransformStamped ros_midcoords, ros_ground_coords, ros_odom_root_coords;
    // ros_midcoords: ROOT -> ground
    // ros_ground_coords: odom -> odom_on_ground
    // ros_odom_root_coords: odom -> odom_root
    // ros_ground_coords: odom_root -> odom_on_ground
    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = parent_frame_id_;
    ros_midcoords.header = header;
    ros_midcoords.child_frame_id = midcoords_frame_id_;
    ros_ground_coords.header.stamp = stamp;
    ros_ground_coords.header.frame_id = odom_root_frame_id_;
    ros_ground_coords.child_frame_id = output_frame_id_;
    ros_odom_root_coords.header.stamp = stamp;
    ros_odom_root_coords.header.frame_id = parent_frame_id_;
    ros_odom_root_coords.child_frame_id = odom_root_frame_id_;
    tf::Transform odom_root_to_odom;
    odom_root_to_odom.setOrigin(tf::Vector3(0, 0, 0));
    /* We should take into account pitch and roll only. */
    // root_link_pose := odom -> root
    // root_link_pose_inv := root -> odom
    Eigen::Affine3f root_link_pose_inv;
    
    tf::transformTFToEigen(root_link_pose_.inverse(), root_link_pose_inv);
    float r, p, y;
    tf::Transform root_link_pose_inv_wo_y_tf;
    pcl::getEulerAngles(root_link_pose_inv, r, p, y);
    Eigen::Affine3f root_link_pose_inv_wo_y = Eigen::Affine3f::Identity() * Eigen::AngleAxisf(r, Eigen::Vector3f::UnitX())
      * Eigen::AngleAxisf(p, Eigen::Vector3f::UnitY());
    tf::transformEigenToTF(root_link_pose_inv_wo_y, root_link_pose_inv_wo_y_tf);
    odom_root_to_odom.setRotation(root_link_pose_inv_wo_y_tf.getRotation());
    tf::transformTFToMsg(midcoords_, ros_midcoords.transform);
    tf::transformTFToMsg(ground_transform_, ros_ground_coords.transform);
    tf::transformTFToMsg(odom_root_to_odom.inverse(), ros_odom_root_coords.transform);
    std::vector<geometry_msgs::TransformStamped> tf_transforms;
    tf_transforms.push_back(ros_midcoords);
    tf_transforms.push_back(ros_odom_root_coords);
    tf_transforms.push_back(ros_ground_coords);
    tf_broadcaster_.sendTransform(tf_transforms);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "footcoords");
  jsk_footstep_controller::Footcoords c;
  ros::spin();
}
