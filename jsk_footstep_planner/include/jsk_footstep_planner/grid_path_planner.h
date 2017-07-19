// -*- mode: c++ -*-

#ifndef JSK_FOOTSTEP_PLANNER_GRID_PATH_PLANNER_H_
#define JSK_FOOTSTEP_PLANNER_GRID_PATH_PLANNER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// ros
#include <sensor_msgs/PointCloud2.h>
//#include <dynamic_reconfigure/server.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <jsk_footstep_msgs/FootstepArray.h>
#include <jsk_footstep_msgs/PlanFootstepsAction.h>
//#include <jsk_footstep_planner/FootstepPlannerConfig.h>
#include <jsk_footstep_planner/CollisionBoundingBoxInfo.h>

// pcl
#include <pcl/kdtree/kdtree_flann.h>

// grid path planning
#include "jsk_footstep_planner/grid_perception.h"

namespace jsk_footstep_planner
{
  enum GridPlanningStatus
  {
    NONE,
    OK,
    WARNING,
    ERROR
  };
  /**
   * @brief
   * Actionlib server for footstep planning
   */
  class GridPathPlanner
  {
  public:
    typedef PerceptionGridGraph Graph;
    typedef PerceptionGridMap GridMap;
    typedef GridAStarSolver<PerceptionGridGraph> Solver;

    GridPathPlanner(ros::NodeHandle& nh);

  protected:
    virtual void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void planCB(const jsk_footstep_msgs::PlanFootstepsGoal::ConstPtr& goal);

    virtual bool collisionBoundingBoxInfoService(
      jsk_footstep_planner::CollisionBoundingBoxInfo::Request& req,
      jsk_footstep_planner::CollisionBoundingBoxInfo::Response& res);
    // buildGraph is not thread safe, it is responsible for caller to take care
    // of mutex
    virtual void buildGraph();

    //    virtual void profile(FootstepAStarSolver<FootstepGraph>& solver, FootstepGraph::Ptr graph);

    virtual void publishPointCloud(
      const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
      ros::Publisher& pub,
      const std_msgs::Header& header);

    virtual void publishText(ros::Publisher& pub,
                             const std::string& text,
                             GridPlanningStatus status);

    virtual bool updateCost(GridState::Ptr ptr);
    //solver.openListToPointCloud(open_list_cloud);
    //solver.closeListToPointCloud(close_list_cloud);

    virtual void publishMarker();

    inline virtual void gridToPoint(const int ix, const int iy, Eigen::Vector3f &p)
    {
      Eigen::Vector3f add(map_resolution_ * (ix + 0.5), map_resolution_ * (iy + 0.5), 0);
      p = map_offset_ + add;
    }

    inline virtual void pointToGrid(const Eigen::Vector3f &p, int &ix, int &iy)
    {
      Eigen::Vector3f ipos = p - map_offset_;
      ix = (int)(ipos[0] / map_resolution_);
      iy = (int)(ipos[1] / map_resolution_);
    }
    // profile

    double heuristicDistance(SolverNode<PerceptionGridGraph::State,
                             PerceptionGridGraph>::Ptr node,
                             PerceptionGridGraph::Ptr graph) {
      int ix = node->getState()->indexX();
      int iy = node->getState()->indexY();
      double gx = graph->getGoalState()->indexX() - ix;
      double gy = graph->getGoalState()->indexY() - iy;

      return std::sqrt(gx * gx + gy * gy);
    }

    boost::mutex mutex_;
    actionlib::SimpleActionServer<jsk_footstep_msgs::PlanFootstepsAction> as_;
    jsk_footstep_msgs::PlanFootstepsResult result_;

    ros::Publisher pub_close_list_;
    ros::Publisher pub_open_list_;
    ros::Publisher pub_text_;
    ros::Publisher pub_marker_;

    ros::Subscriber sub_plane_points_; // plane for stepping
    ros::Subscriber sub_obstacle_points_;   // collision detection with environment

    ros::ServiceServer srv_collision_bounding_box_info_;

    pcl::PointCloud<pcl::PointNormal>::Ptr plane_points_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr    obstacle_points_;
    pcl::KdTreeFLANN<pcl::PointNormal>::Ptr plane_tree_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr    obstacle_tree_;

    std::string plane_points_frame_id_;
    std::string obstacle_points_frame_id_;

    PerceptionGridMap::Ptr gridmap_;
    PerceptionGridGraph::Ptr graph_;

    Eigen::Vector3f map_offset_;

    // Parameters
    Eigen::Vector3f collision_bbox_size_;
    Eigen::Affine3f collision_bbox_offset_;

    double map_resolution_;
    double collision_circle_radius_;
    double collision_circle_min_height_;
    double collision_circle_max_height_;

    bool use_obstacle_points_;
    bool use_plane_points_;
  private:
  };
}

#endif
