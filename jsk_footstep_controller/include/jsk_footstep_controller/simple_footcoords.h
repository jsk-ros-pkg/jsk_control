#ifndef JSK_FOOTSTEP_CONTROLLER_SIMPLE_FOOTCOORDS_H_
#define JSK_FOOTSTEP_CONTROLLER_SIMPLE_FOOTCOORDS_H_

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/SetBool.h>
#include <Eigen/Geometry>

namespace jsk_footstep_controller
{
  /*
    SimpleFootcoords
      Published Topics:
        odom_base_footprint(nsv_msgs/Odometry): "odom"->"base_footprint"
      Subscribed Topics:
        odom
      Provided TF Transforms:
        odom.header.frame_id (e.g. "odom")->odom.child_frame_id (e.g. "BODY"): same as the odom topic
        "odom"->"base_footprint": roll and pitch are zero respect to "odom". z = min(`rfoot_frame_id`_z, `lfoot_frame_id`_z). x, y and yaw are the barycenter of `rfoot_frame_id` and `lfoot_frame_id`
      Services:
        ~enable(std_srvs::setBool): start / stop
      Required TF:
        "odom"->odom.header.frame_id
   */
  class SimpleFootcoords
  {
  public:
    SimpleFootcoords();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    bool enableCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
    bool waitForEndEffectorTransformation(const ros::Time& stamp);
  protected:
    bool isActive_;
    Eigen::Affine3d odom_pose_;

    boost::shared_ptr<tf::TransformListener> tfListener_;
    tf::TransformBroadcaster tfBroadcaster_;
    ros::Subscriber odomSub_;
    ros::Publisher odomBaseFootPrintPub_;
    ros::ServiceServer enableService_;

    std::string lfoot_frame_id_;
    std::string rfoot_frame_id_;
  private:
  };
}
#endif

