#ifndef JSK_FOOTSTEP_CONTROLLER_SIMPLE_FOOTCOORDS_H_
#define JSK_FOOTSTEP_CONTROLLER_SIMPLE_FOOTCOORDS_H_

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <Eigen/Geometry>

namespace jsk_footstep_controller
{
  /*
    SimpleFootcoords
      provided tf transforms:
        odom.header.frame_id (e.g. odom)->odom.child_frame_id (e.g. BODY): same as the odom topic
        `root_frame_id`->base_footprint: roll and pitch are zero respect to odom. z = min(`rfoot_frame_id`_z, `lfoot_frame_id`_z). x, y and yaw are the barycenter of `rfoot_frame_id` and `lfoot_frame_id`
      services:
        ~activate(std_srvs::Empty): start providing tf
        ~deactivate(std_srvs::Empty): stop providing tf
   */
  class SimpleFootcoords
  {
  public:
    SimpleFootcoords();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    bool activateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool deactivateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool waitForEndEffectorTransformation(const ros::Time& stamp);
  protected:
    bool isActive_;
    Eigen::Affine3d odom_pose_;

    boost::shared_ptr<tf::TransformListener> tfListener_;
    tf::TransformBroadcaster tfBroadcaster_;
    ros::Subscriber odomSub_;
    ros::ServiceServer activateService_;
    ros::ServiceServer deactivateService_;

    std::string lfoot_frame_id_;
    std::string rfoot_frame_id_;
  private:
  };
}
#endif

