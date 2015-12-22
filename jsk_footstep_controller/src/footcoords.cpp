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
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <tf_conversions/tf_kdl.h>
#include <geometry_msgs/TwistStamped.h>
#include <kdl_conversions/kdl_msg.h>

namespace jsk_footstep_controller
{

  Footcoords::Footcoords():
    diagnostic_updater_(new diagnostic_updater::Updater)
  {
    ros::NodeHandle nh, pnh("~");
    tf_listener_.reset(new tf::TransformListener());
    odom_status_ = UNINITIALIZED;
    odom_pose_ = Eigen::Affine3d::Identity();
    ground_transform_.setRotation(tf::Quaternion(0, 0, 0, 1));
    root_link_pose_.setIdentity();
    midcoords_.setRotation(tf::Quaternion(0, 0, 0, 1));
    estimated_odom_pose_.setRotation(tf::Quaternion(0, 0, 0, 1));
    diagnostic_updater_->setHardwareID("none");
    diagnostic_updater_->add("Support Leg Status", this,
                             &Footcoords::updateLegDiagnostics);

    // read parameter
    pnh.param("alpha", alpha_, 0.1);
    pnh.param("sampling_time_", sampling_time_, 0.2);
    pnh.param("output_frame_id", output_frame_id_,
              std::string("odom_on_ground"));
    pnh.param("zmp_frame_id", zmp_frame_id_,
              std::string("zmp"));

    pnh.param("parent_frame_id", parent_frame_id_, std::string("odom"));
    pnh.param("midcoords_frame_id", midcoords_frame_id_, std::string("ground"));
    pnh.param("root_frame_id", root_frame_id_, std::string("BODY"));
    pnh.param("body_on_odom_frame", body_on_odom_frame_, std::string("body_on_odom"));
    pnh.param("odom_init_frame_id", odom_init_frame_id_, std::string("odom_init"));
    pnh.param("invert_odom_init", invert_odom_init_, true);
    pnh.param("lfoot_frame_id", lfoot_frame_id_,
              std::string("lleg_end_coords"));
    pnh.param("rfoot_frame_id", rfoot_frame_id_,
              std::string("rleg_end_coords"));
    pnh.param("publish_odom_tf", publish_odom_tf_, true);
    urdf::Model robot_model;
    KDL::Tree tree;
    robot_model.initParam("robot_description");
    if (!kdl_parser::treeFromUrdfModel(robot_model, tree)) {
      ROS_FATAL("Failed to load robot_description");
      return;
    }

    tree.getChain(root_frame_id_, lfoot_frame_id_, lfoot_chain_);
    tree.getChain(root_frame_id_, rfoot_frame_id_, rfoot_chain_);
    for (size_t i=0; i < lfoot_chain_.getNrOfSegments(); i++){
      ROS_INFO_STREAM("kdl_chain(" << i << ") "
                      << lfoot_chain_.getSegment(i).getJoint().getName().c_str());
    }
    for (size_t i=0; i < rfoot_chain_.getNrOfSegments(); i++){
      ROS_INFO_STREAM("kdl_chain(" << i << ") "
                      << rfoot_chain_.getSegment(i).getJoint().getName().c_str());
    }
    pnh.param("lfoot_sensor_frame", lfoot_sensor_frame_, std::string("lleg_end_coords"));
    pnh.param("rfoot_sensor_frame", rfoot_sensor_frame_, std::string("rleg_end_coords"));
    // pnh.param("lfoot_sensor_frame", lfoot_sensor_frame_, std::string("lfsensor"));
    // pnh.param("rfoot_sensor_frame", rfoot_sensor_frame_, std::string("rfsensor"));
    pnh.param("force_threshold", force_thr_, 200.0);
    support_status_ = AIR;
    pub_low_level_ = pnh.advertise<jsk_footstep_controller::FootCoordsLowLevelInfo>("low_level_info", 1);
    pub_state_ = pnh.advertise<std_msgs::String>("state", 1);
    pub_contact_state_ = pnh.advertise<jsk_footstep_controller::GroundContactState>("contact_state", 1);
    pub_synchronized_forces_ = pnh.advertise<jsk_footstep_controller::SynchronizedForces>("synchronized_forces", 1);
    pub_debug_lfoot_pos_ = pnh.advertise<geometry_msgs::Pose>("debug/lfoot_pose", 1);
    pub_debug_rfoot_pos_ = pnh.advertise<geometry_msgs::Pose>("debug/rfoot_pose", 1);
    pub_leg_odometory_ = pnh.advertise<geometry_msgs::PoseStamped>("leg_odometry", 1);
    pub_twist_ = pnh.advertise<geometry_msgs::TwistStamped>("base_vel", 1);
    pub_odom_init_transform_ = pnh.advertise<geometry_msgs::TransformStamped>("odom_init_transform", 1, true);
    pub_odom_init_pose_stamped_ = pnh.advertise<geometry_msgs::PoseStamped>("odom_init_pose_stamped", 1, true);
    before_on_the_air_ = true;
    pnh.param("use_imu", use_imu_, false);
    pnh.param("use_imu_yaw", use_imu_yaw_, true);
    if (publish_odom_tf_) {
      odom_sub_.subscribe(pnh, "/odom", 50);
      imu_sub_.subscribe(pnh, "/imu", 50);
      odom_imu_sync_ = boost::make_shared<message_filters::Synchronizer<OdomImuSyncPolicy> >(100);
      odom_imu_sync_->connectInput(odom_sub_, imu_sub_);
      odom_imu_sync_->registerCallback(boost::bind(&Footcoords::odomImuCallback, this, _1, _2));
    }
    odom_init_pose_ = Eigen::Affine3d::Identity();
    odom_init_trigger_sub_ = pnh.subscribe("/odom_init_trigger", 1,
                                           &Footcoords::odomInitTriggerCallback, this);
    periodic_update_timer_ = pnh.createTimer(ros::Duration(1.0 / 25),
                                             boost::bind(&Footcoords::periodicTimerCallback, this, _1));
    sub_lfoot_force_.subscribe(nh, "lfsensor", 50);
    sub_rfoot_force_.subscribe(nh, "rfsensor", 50);
    sub_joint_states_.subscribe(nh, "joint_states",50);
    sub_zmp_.subscribe(nh, "zmp", 50);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_lfoot_force_, sub_rfoot_force_, sub_joint_states_, sub_zmp_);
    sync_->registerCallback(boost::bind(&Footcoords::synchronizeForces, this, _1, _2, _3, _4));
    synchronized_forces_sub_ = pnh.subscribe("synchronized_forces", 20,
                                             &Footcoords::filter, this);
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

  bool Footcoords::resolveForceTf(const geometry_msgs::WrenchStamped& lfoot,
                                  const geometry_msgs::WrenchStamped& rfoot,
                                  tf::Vector3& lfoot_force,
                                  tf::Vector3& rfoot_force)
  {
    try {
      if (!waitForEndEffectorTrasnformation(lfoot.header.stamp)) {
        ROS_ERROR("[Footcoords::resolveForceTf] failed to lookup transformation for sensor value");
        return false;
      }
      tf::StampedTransform lfoot_transform, rfoot_transform;
      tf_listener_->lookupTransform(
        lfoot.header.frame_id, lfoot_sensor_frame_, lfoot.header.stamp, lfoot_transform);
      tf_listener_->lookupTransform(
        rfoot.header.frame_id, rfoot_sensor_frame_, rfoot.header.stamp, rfoot_transform);
      // cancel translation
      lfoot_transform.setOrigin(tf::Vector3(0, 0, 0));
      rfoot_transform.setOrigin(tf::Vector3(0, 0, 0));

      tf::Vector3 lfoot_local, rfoot_local;
      tf::vector3MsgToTF(lfoot.wrench.force, lfoot_local);
      tf::vector3MsgToTF(rfoot.wrench.force, rfoot_local);
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
    //if (success_to_update) {
    tf::StampedTransform root_transform;
    try {
      // tf_listener_->lookupTransform(parent_frame_id_, root_frame_id_, event.current_real, root_transform);
      // root_link_pose_.setOrigin(root_transform.getOrigin());
      // root_link_pose_.setRotation(root_transform.getRotation());
    }
    catch (tf2::ConnectivityException &e) {
      ROS_ERROR("[Footcoords::resolveForceTf] transform error: %s", e.what());
    }
    catch (tf2::InvalidArgumentException &e) {
      ROS_ERROR("[Footcoords::resolveForceTf] transform error: %s", e.what());
    }
    catch (tf2::ExtrapolationException &e) {
      ROS_ERROR("[Footcoords::resolveForceTf] transform error: %s", e.what());
    }
    catch (tf2::LookupException &e) {
      ROS_ERROR("[Footcoords::resolveForceTf] transform error: %s", e.what());
    }
      //}

      // midcoords is a center coordinates of two feet
      // compute root_link -> midcoords
      // root_link_pose_ := odom -> root_link
      // midcoords_ := odom -> midcoords
      // root_link_pose_ * T = midcoords_
      // T = root_link_pose_^-1 * midcoords_
    //ground_transform_ = root_link_pose_.inverse() * midcoords_;
    ground_transform_ = midcoords_;
      publishTF(event.current_real);
      diagnostic_updater_->update();
      publishContactState(event.current_real);
  }

  void Footcoords::synchronizeForces(const geometry_msgs::WrenchStamped::ConstPtr& lfoot,
                                     const geometry_msgs::WrenchStamped::ConstPtr& rfoot,
                                     const sensor_msgs::JointState::ConstPtr& joint_states,
                                     const geometry_msgs::PointStamped::ConstPtr& zmp)
  {
    jsk_footstep_controller::SynchronizedForces forces;
    forces.header = lfoot->header;
    forces.lleg_force = *lfoot;
    forces.rleg_force = *rfoot;
    forces.joint_angles = *joint_states;
    forces.zmp = *zmp;
    pub_synchronized_forces_.publish(forces);
    Eigen::Vector3d zmp_point;
    tf::pointMsgToEigen(zmp->point, zmp_point);
    // Just for visualization
    if (zmp->point.z < 0) {
      geometry_msgs::TransformStamped ros_zmp_coords;
      ros_zmp_coords.header = zmp->header;
      ros_zmp_coords.child_frame_id = zmp_frame_id_;
      Eigen::Affine3d zmp_pose = Eigen::Affine3d::Identity() * Eigen::Translation3d(zmp_point);        
      tf::transformEigenToMsg(zmp_pose, ros_zmp_coords.transform);
      std::vector<geometry_msgs::TransformStamped> tf_transforms;
      tf_transforms.push_back(ros_zmp_coords);
      {
        boost::mutex::scoped_lock lock(mutex_);
        tf_broadcaster_.sendTransform(tf_transforms);
      }
    }
  }
  
  void Footcoords::updateChain(std::map<std::string, double>& joint_angles,
                               KDL::Chain& chain,
                               tf::Pose& output_pose)
  {
    KDL::JntArray jnt_pos(chain.getNrOfJoints());
    for (int i = 0, j = 0; i < chain.getNrOfSegments(); i++) {
      /*
       * if (chain.getSegment(i).getJoint().getType() == KDL::Joint::None)
       *   continue;
       */
      std::string joint_name = chain.getSegment(i).getJoint().getName();
      if (joint_angles.find(joint_name) != joint_angles.end()) {
        jnt_pos(j++) = joint_angles[joint_name];
      }
    }
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::Frame pose;
    if (fk_solver.JntToCart(jnt_pos, pose) < 0) {
      ROS_FATAL("Failed to compute FK");
    }
    tf::poseKDLToTF(pose, output_pose);
  }

  void Footcoords::updateRobotModel(std::map<std::string, double>& joint_angles)
  {

    updateChain(joint_angles, lfoot_chain_, lfoot_pose_);
    updateChain(joint_angles, rfoot_chain_, rfoot_pose_);
    geometry_msgs::Pose ros_lfoot_pose, ros_rfoot_pose;
    tf::poseTFToMsg(lfoot_pose_, ros_lfoot_pose);
    tf::poseTFToMsg(rfoot_pose_, ros_rfoot_pose);
    pub_debug_lfoot_pos_.publish(ros_lfoot_pose);
    pub_debug_rfoot_pos_.publish(ros_rfoot_pose);
  }

  void Footcoords::computeVelicity(double dt,
                                   std::map<std::string, double>& joint_angles,
                                   KDL::Chain& chain,
                                   geometry_msgs::Twist& output)
  {
    KDL::ChainFkSolverVel_recursive fk_solver(chain);
    KDL::JntArrayVel jnt_pos(chain.getNrOfJoints());
    KDL::JntArray q(chain.getNrOfJoints()), qdot(chain.getNrOfJoints());
    for (int i = 0, j = 0; i < chain.getNrOfSegments(); i++) {
      std::string joint_name = chain.getSegment(i).getJoint().getName();
      if (joint_angles.find(joint_name) != joint_angles.end()) {
        qdot(j) = (joint_angles[joint_name] - prev_joints_[joint_name]) / dt;
        q(j++) = joint_angles[joint_name];
      }
    }
    jnt_pos.q = q;
    jnt_pos.qdot = qdot;
    KDL::FrameVel vel;
    if (fk_solver.JntToCart(jnt_pos, vel) < 0) {
      ROS_FATAL("Failed to compute velocity");
    }
    KDL::Twist twist = vel.GetTwist();
    tf::twistKDLToMsg(twist, output);
  }

  void Footcoords::estimateVelocity(const ros::Time& stamp, std::map<std::string, double>& joint_angles)
  {
    geometry_msgs::TwistStamped twist_stamped;
    if (!prev_joints_.empty()) {
      double dt = (stamp - last_time_).toSec();
      if (odom_status_ == LLEG_SUPPORT || odom_status_ == INITIALIZING) {
        computeVelicity(dt, joint_angles, lfoot_chain_, twist_stamped.twist);
      }
      else if (odom_status_ == RLEG_SUPPORT) {
        computeVelicity(dt, joint_angles, rfoot_chain_, twist_stamped.twist);
      }
      twist_stamped.header.stamp = stamp;
      twist_stamped.header.frame_id = root_frame_id_;
      pub_twist_.publish(twist_stamped);
    }
  }

  void Footcoords::filter(const jsk_footstep_controller::SynchronizedForces::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    geometry_msgs::WrenchStamped lfoot = msg->lleg_force;
    geometry_msgs::WrenchStamped rfoot = msg->rleg_force;
    zmp_ = Eigen::Vector3f(msg->zmp.point.x, msg->zmp.point.y, msg->zmp.point.z);
    sensor_msgs::JointState joint_angles = msg->joint_angles;
    std::map<std::string, double> angles;
    for (size_t i = 0; i < joint_angles.name.size(); i++) {
      angles[joint_angles.name[i]] = joint_angles.position[i];
    }

    updateRobotModel(angles);

    // lowpass filter
    tf::Vector3 lfoot_force_vector, rfoot_force_vector;
    if (!resolveForceTf(lfoot, rfoot, lfoot_force_vector, rfoot_force_vector)) {
      ROS_ERROR("[Footcoords::filter] failed to resolve tf of force sensor");
      return;
    }
    // resolve tf
    double lfoot_force = applyLowPassFilter(lfoot_force_vector[2], prev_lforce_);
    double rfoot_force = applyLowPassFilter(rfoot_force_vector[2], prev_rforce_);

    // publish lowlevel info
    FootCoordsLowLevelInfo info;
    info.header = lfoot.header;
    info.raw_lleg_force = lfoot_force_vector[2];
    info.raw_rleg_force = rfoot_force_vector[2];
    info.filtered_lleg_force = lfoot_force;
    info.filtered_rleg_force = rfoot_force;
    info.threshold = force_thr_;
    pub_low_level_.publish(info);
    prev_lforce_ = lfoot_force;
    prev_rforce_ = rfoot_force;

    if (lfoot_force >= force_thr_ && rfoot_force >= force_thr_) {
      support_status_ = BOTH_GROUND;
    }
    else if (lfoot_force < force_thr_ && rfoot_force < force_thr_) {
      support_status_ = AIR;
    }
    else if (lfoot_force >= force_thr_) {
      // only left
      support_status_ = LLEG_GROUND;
    }
    else if (rfoot_force >= force_thr_) {
      // only right
      support_status_ = RLEG_GROUND;
    }
    estimateOdometry();
    estimateVelocity(msg->header.stamp, angles);
    geometry_msgs::PoseStamped leg_odometory;
    leg_odometory.header.stamp = msg->header.stamp;
    leg_odometory.header.frame_id = root_frame_id_;
    tf::poseTFToMsg(estimated_odom_pose_, leg_odometory.pose);
    pub_leg_odometory_.publish(leg_odometory);
    /* tf for debug*/
    std::vector<geometry_msgs::TransformStamped> tf_transforms;
    geometry_msgs::TransformStamped odom_transform;
    tf::transformTFToMsg(estimated_odom_pose_, odom_transform.transform);
    odom_transform.header = leg_odometory.header;
    odom_transform.child_frame_id = "leg_odom";
    tf_transforms.push_back(odom_transform);
    tf_broadcaster_.sendTransform(tf_transforms);
    prev_joints_ = angles;
    last_time_ = msg->header.stamp;
  }

  void Footcoords::estimateOdometry()
  {
    //estimateOdometryMainSupportLeg();
    estimateOdometryZMPSupportLeg();
  }

  void Footcoords::estimateOdometryNaive()
  {
    if (odom_status_ == UNINITIALIZED && support_status_ == BOTH_GROUND) {
      ROS_INFO("changed to INITIALIZING");
      odom_status_ = INITIALIZING;
    }
    if (odom_status_ == INITIALIZING && support_status_ == BOTH_GROUND) {
      ROS_INFO_THROTTLE(1.0, "INITIALIZING");
      /* Update odom origin */
      tf::Quaternion midcoords_rot = lfoot_pose_.getRotation().slerp(rfoot_pose_.getRotation(), 0.5);
      tf::Vector3 midcoords_pos = lfoot_pose_.getOrigin().lerp(rfoot_pose_.getOrigin(), 0.5);
      tf::Pose midcoords;
      midcoords.setOrigin(midcoords_pos);
      midcoords.setRotation(midcoords_rot);
      lfoot_to_origin_pose_ = lfoot_pose_.inverse() * midcoords;
      rfoot_to_origin_pose_ = rfoot_pose_.inverse() * midcoords;
      estimated_odom_pose_ = midcoords;
    }
    if (odom_status_ == INITIALIZING && support_status_ ==RLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "detect RLEG support phase");
      odom_status_ = RLEG_SUPPORT;
      estimated_odom_pose_ = rfoot_pose_ * rfoot_to_origin_pose_;
    }
    else if (odom_status_ == INITIALIZING && support_status_ == LLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "detect LLEG support phase");
      odom_status_ = LLEG_SUPPORT;
      estimated_odom_pose_ = lfoot_pose_ * lfoot_to_origin_pose_;
    }
    else if (odom_status_ == RLEG_SUPPORT && support_status_ == RLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "keep RLEG support phase");
      estimated_odom_pose_ = rfoot_pose_ * rfoot_to_origin_pose_;
    }
    else if (odom_status_ == RLEG_SUPPORT && support_status_ == BOTH_GROUND) {
      ROS_INFO_THROTTLE(1.0, "dual leg support phase but use RLEG");
      estimated_odom_pose_ = rfoot_pose_ * rfoot_to_origin_pose_;
      lfoot_to_origin_pose_ = lfoot_pose_.inverse() * rfoot_pose_ * rfoot_to_origin_pose_;
    }
    else if (odom_status_ == RLEG_SUPPORT && support_status_ == LLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "detect LLEG support phase");
      odom_status_ = LLEG_SUPPORT;
      estimated_odom_pose_ = lfoot_pose_ * lfoot_to_origin_pose_;
    }
    else if (odom_status_ == LLEG_SUPPORT && support_status_ == LLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "keep LLEG support phase");
      estimated_odom_pose_ = lfoot_pose_ * lfoot_to_origin_pose_;;
    }
    else if (odom_status_ == LLEG_SUPPORT && support_status_ == BOTH_GROUND) {
      ROS_INFO_THROTTLE(1.0, "dual leg support phase but use LLEG");
      estimated_odom_pose_ = lfoot_pose_ * lfoot_to_origin_pose_;;
      rfoot_to_origin_pose_ = rfoot_pose_.inverse() * lfoot_pose_ * lfoot_to_origin_pose_;
    }
    else if (odom_status_ == LLEG_SUPPORT && support_status_ == RLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "detect RLEG support phase");
      odom_status_ = RLEG_SUPPORT;
      estimated_odom_pose_ = rfoot_pose_ * rfoot_to_origin_pose_;
    }
    if (support_status_ == AIR) {
      ROS_INFO_THROTTLE(1.0, "resetting");
      odom_status_ = UNINITIALIZED;
    }
  }

  void Footcoords::estimateOdometryMainSupportLeg()
  {
    if (odom_status_ == UNINITIALIZED && support_status_ == BOTH_GROUND) {
      ROS_INFO("changed to INITIALIZING");
      odom_status_ = INITIALIZING;
    }
    if (odom_status_ == INITIALIZING && support_status_ == BOTH_GROUND) {
      ROS_INFO_THROTTLE(1.0, "INITIALIZING");
      /* Update odom origin */
      tf::Quaternion midcoords_rot = lfoot_pose_.getRotation().slerp(rfoot_pose_.getRotation(), 0.5);
      tf::Vector3 midcoords_pos = lfoot_pose_.getOrigin().lerp(rfoot_pose_.getOrigin(), 0.5);
      tf::Pose midcoords;
      midcoords.setOrigin(midcoords_pos);
      midcoords.setRotation(midcoords_rot);
      lfoot_to_origin_pose_ = lfoot_pose_.inverse() * midcoords;
      rfoot_to_origin_pose_ = rfoot_pose_.inverse() * midcoords;
      estimated_odom_pose_ = midcoords;
    }
    if (odom_status_ == INITIALIZING && support_status_ == RLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "detect RLEG support phase");
      odom_status_ = RLEG_SUPPORT;
      estimated_odom_pose_ = rfoot_pose_ * rfoot_to_origin_pose_;
    }
    else if (odom_status_ == INITIALIZING && support_status_ == LLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "detect LLEG support phase");
      odom_status_ = LLEG_SUPPORT;
      estimated_odom_pose_ = lfoot_pose_ * lfoot_to_origin_pose_;
    }
    else if (odom_status_ == RLEG_SUPPORT && support_status_ ==RLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "keep RLEG support phase");
      estimated_odom_pose_ = rfoot_pose_ * rfoot_to_origin_pose_;
    }
    else if (odom_status_ == RLEG_SUPPORT && support_status_ == BOTH_GROUND) {
      if (prev_lforce_ > prev_rforce_) {
        /* switch to lleg */
        odom_status_ = LLEG_SUPPORT;
        ROS_INFO_THROTTLE(1.0, "dual leg support phase, switch to lleg");
        estimated_odom_pose_ = lfoot_pose_ * lfoot_to_origin_pose_;
        rfoot_to_origin_pose_ = rfoot_pose_.inverse() * lfoot_pose_ * lfoot_to_origin_pose_;
      }
      else {
        ROS_INFO_THROTTLE(1.0, "dual leg support phase but use RLEG");
        estimated_odom_pose_ = rfoot_pose_ * rfoot_to_origin_pose_;
        lfoot_to_origin_pose_ = lfoot_pose_.inverse() * rfoot_pose_ * rfoot_to_origin_pose_;
      }
    }
    else if (odom_status_ == RLEG_SUPPORT && support_status_ == LLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "detect LLEG support phase");
      odom_status_ = LLEG_SUPPORT;
      estimated_odom_pose_ = lfoot_pose_ * lfoot_to_origin_pose_;
    }
    else if (odom_status_ == LLEG_SUPPORT && support_status_ == LLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "keep LLEG support phase");
      estimated_odom_pose_ = lfoot_pose_ * lfoot_to_origin_pose_;;
    }
    else if (odom_status_ == LLEG_SUPPORT && support_status_ == BOTH_GROUND) {
      if (prev_rforce_ > prev_lforce_) {
        odom_status_ = RLEG_SUPPORT;
        ROS_INFO_THROTTLE(1.0, "dual leg support phase, switch to rleg");
        estimated_odom_pose_ = rfoot_pose_ * rfoot_to_origin_pose_;
        lfoot_to_origin_pose_ = lfoot_pose_.inverse() * rfoot_pose_ * rfoot_to_origin_pose_;
      }
      else {
        ROS_INFO_THROTTLE(1.0, "dual leg support phase but use LLEG");
        estimated_odom_pose_ = lfoot_pose_ * lfoot_to_origin_pose_;;
        rfoot_to_origin_pose_ = rfoot_pose_.inverse() * lfoot_pose_ * lfoot_to_origin_pose_;
      }
    }
    else if (odom_status_ == LLEG_SUPPORT && support_status_ == RLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "detect RLEG support phase");
      odom_status_ = RLEG_SUPPORT;
      estimated_odom_pose_ = rfoot_pose_ * rfoot_to_origin_pose_;
    }
    if (support_status_ == AIR) {
      ROS_INFO_THROTTLE(1.0, "resetting");
      odom_status_ = UNINITIALIZED;
    }
  }

    void Footcoords::estimateOdometryZMPSupportLeg()
  {
    if (odom_status_ == UNINITIALIZED && support_status_ == BOTH_GROUND) {
      ROS_INFO("changed to INITIALIZING");
      odom_status_ = INITIALIZING;
    }
    if (odom_status_ == INITIALIZING && support_status_ == BOTH_GROUND) {
      ROS_INFO_THROTTLE(1.0, "INITIALIZING");
      /* Update odom origin */
      tf::Quaternion midcoords_rot = lfoot_pose_.getRotation().slerp(rfoot_pose_.getRotation(), 0.5);
      tf::Vector3 midcoords_pos = lfoot_pose_.getOrigin().lerp(rfoot_pose_.getOrigin(), 0.5);
      tf::Pose midcoords;
      midcoords.setOrigin(midcoords_pos);
      midcoords.setRotation(midcoords_rot);
      lfoot_to_origin_pose_ = lfoot_pose_.inverse() * midcoords;
      rfoot_to_origin_pose_ = rfoot_pose_.inverse() * midcoords;
      estimated_odom_pose_ = midcoords;
    }
    if (odom_status_ == INITIALIZING && support_status_ == RLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "detect RLEG support phase");
      odom_status_ = RLEG_SUPPORT;
      estimated_odom_pose_ = rfoot_pose_ * rfoot_to_origin_pose_;
    }
    else if (odom_status_ == INITIALIZING && support_status_ == LLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "detect LLEG support phase");
      odom_status_ = LLEG_SUPPORT;
      estimated_odom_pose_ = lfoot_pose_ * lfoot_to_origin_pose_;
    }
    else if (odom_status_ == RLEG_SUPPORT && support_status_ ==RLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "keep RLEG support phase");
      estimated_odom_pose_ = rfoot_pose_ * rfoot_to_origin_pose_;
    }
    else if (odom_status_ == RLEG_SUPPORT && support_status_ == BOTH_GROUND) {
      if (zmp_[1] > 0) {
        /* switch to lleg */
        odom_status_ = LLEG_SUPPORT;
        ROS_INFO_THROTTLE(1.0, "dual leg support phase, switch to lleg");
        estimated_odom_pose_ = lfoot_pose_ * lfoot_to_origin_pose_;
        rfoot_to_origin_pose_ = rfoot_pose_.inverse() * lfoot_pose_ * lfoot_to_origin_pose_;
      }
      else {
        ROS_INFO_THROTTLE(1.0, "dual leg support phase but use RLEG");
        estimated_odom_pose_ = rfoot_pose_ * rfoot_to_origin_pose_;
        lfoot_to_origin_pose_ = lfoot_pose_.inverse() * rfoot_pose_ * rfoot_to_origin_pose_;
      }
    }
    else if (odom_status_ == RLEG_SUPPORT && support_status_ == LLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "detect LLEG support phase");
      odom_status_ = LLEG_SUPPORT;
      estimated_odom_pose_ = lfoot_pose_ * lfoot_to_origin_pose_;
    }
    else if (odom_status_ == LLEG_SUPPORT && support_status_ == LLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "keep LLEG support phase");
      estimated_odom_pose_ = lfoot_pose_ * lfoot_to_origin_pose_;;
    }
    else if (odom_status_ == LLEG_SUPPORT && support_status_ == BOTH_GROUND) {
      if (zmp_[1] < 0) {
        odom_status_ = RLEG_SUPPORT;
        ROS_INFO_THROTTLE(1.0, "dual leg support phase, switch to rleg");
        estimated_odom_pose_ = rfoot_pose_ * rfoot_to_origin_pose_;
        lfoot_to_origin_pose_ = lfoot_pose_.inverse() * rfoot_pose_ * rfoot_to_origin_pose_;
      }
      else {
        ROS_INFO_THROTTLE(1.0, "dual leg support phase but use LLEG");
        estimated_odom_pose_ = lfoot_pose_ * lfoot_to_origin_pose_;;
        rfoot_to_origin_pose_ = rfoot_pose_.inverse() * lfoot_pose_ * lfoot_to_origin_pose_;
      }
    }
    else if (odom_status_ == LLEG_SUPPORT && support_status_ == RLEG_GROUND) {
      ROS_INFO_THROTTLE(1.0, "detect RLEG support phase");
      odom_status_ = RLEG_SUPPORT;
      estimated_odom_pose_ = rfoot_pose_ * rfoot_to_origin_pose_;
    }
    if (support_status_ == AIR) {
      ROS_INFO_THROTTLE(1.0, "resetting");
      odom_status_ = UNINITIALIZED;
    }
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
            root_frame_id_, lfoot_frame_id_, stamp, foot_transform);
        }
        else {                  // right on the ground
          tf_listener_->lookupTransform(
            root_frame_id_, rfoot_frame_id_, stamp, foot_transform);
        }
        midcoords_ = foot_transform.inverse();
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
          root_frame_id_, lfoot_frame_id_, stamp, lfoot_transform);
        tf::StampedTransform rfoot_transform;
        tf_listener_->lookupTransform(
          root_frame_id_, rfoot_frame_id_, stamp, rfoot_transform);
        tf::Quaternion lfoot_rot = lfoot_transform.getRotation();
        tf::Quaternion rfoot_rot = rfoot_transform.getRotation();
        tf::Quaternion mid_rot = lfoot_rot.slerp(rfoot_rot, 0.5);
        tf::Vector3 lfoot_pos = lfoot_transform.getOrigin();
        tf::Vector3 rfoot_pos = rfoot_transform.getOrigin();
        tf::Vector3 mid_pos = lfoot_pos.lerp(rfoot_pos, 0.5);
        midcoords_.setOrigin(mid_pos);
        midcoords_.setRotation(mid_rot);
        midcoords_ = midcoords_;
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
          root_frame_id_, lfoot_frame_id_, stamp, ros::Duration(1.0))) {
      ROS_ERROR("[Footcoords::waitForEndEffectorTrasnformation] failed to lookup transform between %s and %s",
                root_frame_id_.c_str(),
                lfoot_frame_id_.c_str());
      return false;
    }
    // odom -> rfoot
    else if (!tf_listener_->waitForTransform(
               root_frame_id_, rfoot_frame_id_, stamp, ros::Duration(1.0))) {
      ROS_ERROR("[Footcoords::waitForEndEffectorTrasnformation]failed to lookup transform between %s and %s",
                root_frame_id_.c_str(),
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

  void Footcoords::getRollPitch(const Eigen::Affine3d& pose, float& roll, float& pitch)
  {
    Eigen::Affine3f posef;
    jsk_pcl_ros::convertEigenAffine3(pose, posef);
    float yaw;
    pcl::getEulerAngles(posef, roll, pitch, yaw);
  }

  float Footcoords::getYaw(const Eigen::Affine3d& pose)
  {
    Eigen::Affine3f posef;
    jsk_pcl_ros::convertEigenAffine3(pose, posef);
    float roll, pitch, yaw;
    pcl::getEulerAngles(posef, roll, pitch, yaw);
    return yaw;
  }

  void Footcoords::publishTF(const ros::Time& stamp)
  {
    // publish midcoords_ and ground_cooords_
    geometry_msgs::TransformStamped ros_midcoords, 
      ros_ground_coords, ros_odom_to_body_coords,
      ros_body_on_odom_coords, ros_odom_init_coords;
    // ros_midcoords: ROOT -> ground
    // ros_ground_coords: odom -> odom_on_ground = identity
    // ros_odom_root_coords: odom -> odom_root = identity
    // ros_ground_coords: odom_root -> odom_on_ground = identity
    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = parent_frame_id_;
    ros_midcoords.header.stamp = stamp;
    ros_midcoords.header.frame_id = root_frame_id_;
    ros_midcoords.child_frame_id = midcoords_frame_id_;
    ros_ground_coords.header.stamp = stamp;
    ros_ground_coords.header.frame_id = parent_frame_id_;
    ros_ground_coords.child_frame_id = output_frame_id_;
    ros_odom_to_body_coords.header.stamp = stamp;
    ros_odom_to_body_coords.header.frame_id = parent_frame_id_;
    ros_odom_to_body_coords.child_frame_id = root_frame_id_;
    ros_body_on_odom_coords.header.stamp = stamp;
    ros_body_on_odom_coords.header.frame_id = root_frame_id_;
    ros_body_on_odom_coords.child_frame_id = body_on_odom_frame_;
    ros_odom_init_coords.header.stamp = stamp;
    if (invert_odom_init_) {
      ros_odom_init_coords.header.frame_id = odom_init_frame_id_;
      ros_odom_init_coords.child_frame_id = parent_frame_id_;
    } else {
      ros_odom_init_coords.header.frame_id = parent_frame_id_;
      ros_odom_init_coords.child_frame_id = odom_init_frame_id_;
    }
    Eigen::Affine3d identity = Eigen::Affine3d::Identity();
    float roll, pitch;
    getRollPitch(odom_pose_, roll, pitch);
    Eigen::Affine3d body_on_odom_pose = (Eigen::Translation3d(0,
                                                              0,
                                                              odom_pose_.translation()[2]) * 
                                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
    midcoords_.getRotation().normalize();
    Eigen::Affine3d odom_init_pose = (Eigen::Translation3d(odom_init_pose_.translation()[0],
                                                           odom_init_pose_.translation()[1],
                                                           0.0) * 
                                      Eigen::AngleAxisd(getYaw(odom_init_pose_), Eigen::Vector3d::UnitZ()));

    tf::transformTFToMsg(midcoords_, ros_midcoords.transform);
    tf::transformEigenToMsg(identity, ros_ground_coords.transform);
    tf::transformEigenToMsg(body_on_odom_pose.inverse(), ros_body_on_odom_coords.transform);
    tf::transformEigenToMsg(odom_pose_, ros_odom_to_body_coords.transform);
    if (invert_odom_init_) {
      tf::transformEigenToMsg(odom_init_pose.inverse(), ros_odom_init_coords.transform);
    } else {
      tf::transformEigenToMsg(odom_init_pose, ros_odom_init_coords.transform);
    }
    std::vector<geometry_msgs::TransformStamped> tf_transforms;
    tf_transforms.push_back(ros_midcoords);
    tf_transforms.push_back(ros_ground_coords);
    tf_transforms.push_back(ros_odom_to_body_coords);
    tf_transforms.push_back(ros_body_on_odom_coords);
    tf_transforms.push_back(ros_odom_init_coords);
    tf_broadcaster_.sendTransform(tf_transforms);
  }

  void Footcoords::odomInitTriggerCallback(const std_msgs::Empty& trigger)
  {
    boost::mutex::scoped_lock lock(mutex_);
    // Update odom_init_pose
    odom_init_pose_ = odom_pose_;
    
    // publish odom_init topics
    // whether invert_odom_init is true or not odom_init_pose_stamped and odom_init_transform is described in odom coordinates.
    geometry_msgs::TransformStamped ros_odom_init_coords;
    geometry_msgs::PoseStamped ros_odom_init_pose_stamped;
    Eigen::Affine3d odom_init_pose = (Eigen::Translation3d(odom_init_pose_.translation()[0],
                                                           odom_init_pose_.translation()[1],
                                                           0.0) * 
                                      Eigen::AngleAxisd(getYaw(odom_init_pose_), Eigen::Vector3d::UnitZ()));
    ros_odom_init_coords.header.stamp = ros::Time::now();
    ros_odom_init_coords.header.frame_id = parent_frame_id_;
    ros_odom_init_coords.child_frame_id = odom_init_frame_id_;
    tf::transformEigenToMsg(odom_init_pose, ros_odom_init_coords.transform);
    pub_odom_init_transform_.publish(ros_odom_init_coords);
    ros_odom_init_pose_stamped.header = ros_odom_init_coords.header;
    ros_odom_init_pose_stamped.pose.position.x = ros_odom_init_coords.transform.translation.x;
    ros_odom_init_pose_stamped.pose.position.y = ros_odom_init_coords.transform.translation.y;
    ros_odom_init_pose_stamped.pose.position.z = ros_odom_init_coords.transform.translation.z;
    ros_odom_init_pose_stamped.pose.orientation = ros_odom_init_coords.transform.rotation;
    pub_odom_init_pose_stamped_.publish(ros_odom_init_pose_stamped);
  }

  void Footcoords::odomImuCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                                   const sensor_msgs::Imu::ConstPtr& imu_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (use_imu_) {
      Eigen::Affine3d odom_pose;
      tf::poseMsgToEigen(odom_msg->pose.pose, odom_pose);
      Eigen::Quaterniond imu_orientation;
      tf::quaternionMsgToEigen(imu_msg->orientation, imu_orientation);
      if (use_imu_yaw_) {
        odom_pose_ = Eigen::Translation3d(odom_pose.translation()) * imu_orientation;
      }
      else {
        float roll, pitch;
        getRollPitch(Eigen::Affine3d::Identity() * imu_orientation, roll, pitch);
        odom_pose_ = (Eigen::Translation3d(odom_pose.translation()) *
                      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
      }
    }
    else {
      tf::poseMsgToEigen(odom_msg->pose.pose, odom_pose_);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "footcoords");
  jsk_footstep_controller::Footcoords c;
  ros::spin();
}
