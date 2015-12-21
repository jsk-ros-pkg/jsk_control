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

#ifndef JSK_FOOTSTEP_CONTROLLER_FOOTCOORDS_H_
#define JSK_FOOTSTEP_CONTROLLER_FOOTCOORDS_H_
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_listener.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <jsk_footstep_controller/GroundContactState.h>
#include <jsk_footstep_controller/FootCoordsLowLevelInfo.h>
#include <jsk_footstep_controller/SynchronizedForces.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace jsk_footstep_controller
{
  template <class T>
  class TimeStampedVector: public std::vector<T>
  {
  public:
    typedef typename std::vector<T>::iterator iterator;
    void removeBefore(const ros::Time& stamp)
    {
      for (iterator it = std::vector<T>::begin();
           it != std::vector<T>::end();) {
        if (((*it)->header.stamp - stamp) < ros::Duration(0.0)) {
          it = this->erase(it);
        }
        else {
          ++it;
        }
      }
    }
  protected:
  private:
  };


  class ValueStamped
  {
  public:
    typedef boost::shared_ptr<ValueStamped> Ptr;
    std_msgs::Header header;
    double value;
    ValueStamped(const std_msgs::Header& aheader, double avalue):
      header(aheader), value(avalue) {
    }
    ValueStamped() { }
  };

  class Footcoords
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
    geometry_msgs::WrenchStamped,
    geometry_msgs::WrenchStamped,
    sensor_msgs::JointState,
    geometry_msgs::PointStamped> SyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      nav_msgs::Odometry,
      sensor_msgs::Imu> OdomImuSyncPolicy;

    Footcoords();
    virtual ~Footcoords();

    enum SupportLegStatus
    {
      LLEG_GROUND, RLEG_GROUND, AIR, BOTH_GROUND
    };

    enum OdomStatus
    {
      UNINITIALIZED, INITIALIZING, LLEG_SUPPORT, RLEG_SUPPORT
    };

  protected:
    
    // methods

    /* 
     * virtual void filter(const geometry_msgs::WrenchStamped::ConstPtr& lfoot,
     *                     const geometry_msgs::WrenchStamped::ConstPtr& rfoot);
     */
    virtual void filter(const jsk_footstep_controller::SynchronizedForces::ConstPtr& msg);
    virtual bool computeMidCoords(const ros::Time& stamp);
    virtual bool computeMidCoordsFromSingleLeg(const ros::Time& stamp,
                                               bool use_left_leg);
    virtual bool waitForEndEffectorTrasnformation(const ros::Time& stamp);
    virtual bool waitForSensorFrameTransformation(const ros::Time& stamp,
                                                  const std::string& lsensor_frame,
                                                  const std::string& rsensor_frame);
    virtual bool updateGroundTF();
    virtual void publishTF(const ros::Time& stamp);
    virtual void publishState(const std::string& state);
    virtual void updateLegDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual void publishContactState(const ros::Time& stamp);
    virtual double applyLowPassFilter(double current_val, double prev_val) const;
    virtual bool allValueLargerThan(TimeStampedVector<ValueStamped::Ptr>& values,
                                    double threshold);
    virtual bool allValueSmallerThan(TimeStampedVector<ValueStamped::Ptr>& values,
                                     double threshold);
    virtual bool resolveForceTf(const geometry_msgs::WrenchStamped& lfoot,
                                const geometry_msgs::WrenchStamped& rfoot,
                                tf::Vector3& lfoot_force, tf::Vector3& rfoot_force);
    virtual void periodicTimerCallback(const ros::TimerEvent& event);
    virtual void odomInitTriggerCallback(const std_msgs::Empty& trigger);
    virtual void odomImuCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                                 const sensor_msgs::Imu::ConstPtr& imu_msg);
    // This callback is called when robot is put on the ground.
    // virtual void odomInitCallback(const std_msgs::Empty& odom_init_msg);
    virtual void synchronizeForces(const geometry_msgs::WrenchStamped::ConstPtr& lfoot,
                                   const geometry_msgs::WrenchStamped::ConstPtr& rfoot,
                                   const sensor_msgs::JointState::ConstPtr& joint_states,
                                   const geometry_msgs::PointStamped::ConstPtr& zmp);
    virtual void estimateOdometry();
    virtual void estimateOdometryNaive();
    virtual void estimateOdometryMainSupportLeg();
    virtual void estimateOdometryZMPSupportLeg();
    virtual void updateRobotModel(std::map<std::string, double>& joint_angles);
    virtual void updateChain(std::map<std::string, double>& joint_angles,
                             KDL::Chain& chain,
                             tf::Pose& output_pose);
    virtual void estimateVelocity(const ros::Time& stamp, std::map<std::string, double>& joint_angles);
    virtual void computeVelicity(double dt,
                                 std::map<std::string, double>& joint_angles,
                                 KDL::Chain& chain,
                                 geometry_msgs::Twist& output);
    virtual float getYaw(const Eigen::Affine3d& pose);
    virtual void getRollPitch(const Eigen::Affine3d& pose, float& roll, float& pitch);
    // ros variables
    boost::mutex mutex_;
    Eigen::Affine3d odom_pose_;
    Eigen::Affine3d odom_init_pose_;
    ros::Timer periodic_update_timer_;
    ros::Subscriber odom_init_trigger_sub_;
    //ros::Subscriber odom_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    boost::shared_ptr<message_filters::Synchronizer<OdomImuSyncPolicy> > odom_imu_sync_;
    ros::Subscriber synchronized_forces_sub_;
    message_filters::Subscriber<geometry_msgs::WrenchStamped> sub_lfoot_force_;
    message_filters::Subscriber<geometry_msgs::WrenchStamped> sub_rfoot_force_;
    message_filters::Subscriber<sensor_msgs::JointState> sub_joint_states_;
    message_filters::Subscriber<geometry_msgs::PointStamped> sub_zmp_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    KDL::Chain lfoot_chain_;
    KDL::Chain rfoot_chain_;
    tf::Pose lfoot_pose_;
    tf::Pose rfoot_pose_;
    tf::Pose lfoot_to_origin_pose_;
    tf::Pose rfoot_to_origin_pose_;
    tf::Pose estimated_odom_pose_;
    ros::Publisher pub_state_;
    ros::Publisher pub_contact_state_;
    ros::Publisher pub_low_level_;
    ros::Publisher pub_synchronized_forces_;
    ros::Publisher pub_debug_lfoot_pos_;
    ros::Publisher pub_debug_rfoot_pos_;
    ros::Publisher pub_leg_odometory_;
    ros::Publisher pub_twist_;
    ros::Publisher pub_odom_init_transform_;
    ros::Publisher pub_odom_init_pose_stamped_;
    boost::shared_ptr<tf::TransformListener> tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    // parameters
    std::string zmp_frame_id_;
    std::string output_frame_id_;
    std::string parent_frame_id_;
    std::string midcoords_frame_id_;
    SupportLegStatus support_status_;
    OdomStatus odom_status_;
    Eigen::Vector3f zmp_;
    double force_thr_;
    bool before_on_the_air_;
    std::map<std::string, double> prev_joints_;
    bool use_imu_;
    bool use_imu_yaw_;
    ros::Time last_time_;
    std::string lfoot_frame_id_;
    std::string rfoot_frame_id_;
    std::string lfoot_sensor_frame_;
    std::string rfoot_sensor_frame_;
    std::string root_frame_id_;
    std::string body_on_odom_frame_;
    std::string odom_init_frame_id_;
    bool invert_odom_init_;
    tf::Transform ground_transform_;
    tf::Transform midcoords_;
    tf::Transform root_link_pose_;
    tf::Transform locked_midcoords_to_odom_on_ground_;
    boost::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
    double prev_lforce_;
    double prev_rforce_;
    double alpha_;
    double sampling_time_;
    bool publish_odom_tf_;
  private:
  };
}
#endif
