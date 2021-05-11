// -*- mode: c++ -*-

#include "jsk_footstep_planner/grid_path_planner.h"

//param
#include <jsk_topic_tools/rosparam_utils.h>

#include <jsk_recognition_utils/pcl_conversion_util.h> // kdtree

#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_utils/geo/polyline.h>

namespace jsk_footstep_planner
{
  GridPathPlanner::GridPathPlanner(ros::NodeHandle& nh):
    as_(nh, nh.getNamespace(),
        boost::bind(&GridPathPlanner::planCB, this, _1), false),
    plane_tree_(new pcl::KdTreeFLANN<pcl::PointNormal>),
    obstacle_tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>)
  {
    pub_text_ = nh.advertise<jsk_rviz_plugins::OverlayText> ("text", 1, true);
    pub_marker_ = nh.advertise<visualization_msgs::MarkerArray> ("grid_graph_marker", 2, true);
    pub_close_list_ = nh.advertise<sensor_msgs::PointCloud2> ("close_list", 1, true);
    pub_open_list_  = nh.advertise<sensor_msgs::PointCloud2> ("open_list", 1, true);

    srv_collision_bounding_box_info_ = nh.advertiseService(
      "collision_bounding_box_info", &GridPathPlanner::collisionBoundingBoxInfoService, this);

    sub_plane_points_ = nh.subscribe("plane_points", 1, &GridPathPlanner::pointcloudCallback, this);
    sub_obstacle_points_   = nh.subscribe("obstacle_points", 1, &GridPathPlanner::obstacleCallback, this);

    std::vector<double> collision_bbox_size, collision_bbox_offset;
    if (jsk_topic_tools::readVectorParameter(nh, "collision_bbox_size", collision_bbox_size)) {
      collision_bbox_size_[0] = collision_bbox_size[0];
      collision_bbox_size_[1] = collision_bbox_size[1];
      collision_bbox_size_[2] = collision_bbox_size[2];
    }
    if (jsk_topic_tools::readVectorParameter(nh, "collision_bbox_offset", collision_bbox_offset)) {
      collision_bbox_offset_ = Eigen::Affine3f::Identity() * Eigen::Translation3f(collision_bbox_offset[0],
                                                                                  collision_bbox_offset[1],
                                                                                  collision_bbox_offset[2]);
    }

    nh.param("map_resolution", map_resolution_, 0.4);
    ROS_INFO("map resolution: %f", map_resolution_);
    nh.param("collision_circle_radius", collision_circle_radius_, 0.35);
    nh.param("collision_circle_min_height", collision_circle_min_height_, 0.4);
    nh.param("collision_circle_max_height", collision_circle_max_height_, 1.9);
    use_obstacle_points_ = true;
    use_plane_points_ = true;

    as_.start();
  }

  void GridPathPlanner::buildGraph()
  {
    //boost::mutex::scoped_lock lock(mutex_);
    ROS_INFO("build plane %d", plane_points_->points.size());
    Eigen::Vector4f minpt, maxpt;
    pcl::getMinMax3D<pcl::PointNormal> (*plane_points_, minpt, maxpt);

    Eigen::Vector4f len = maxpt - minpt;
    map_offset_[0] = minpt[0];
    map_offset_[1] = minpt[1];
    map_offset_[2] = 0.0;
    int sizex = (int)(len[0] / map_resolution_);
    int sizey = (int)(len[1] / map_resolution_);

    ROS_INFO("min_point: [%f, %f] / size: %d %d",
             map_offset_[0], map_offset_[1], sizex, sizey);

    obstacle_tree_->setInputCloud(obstacle_points_);
    plane_tree_->setInputCloud(plane_points_);

    gridmap_.reset(new GridMap(sizex, sizey));
    gridmap_->setCostFunction(boost::bind(&GridPathPlanner::updateCost, this, _1));
    graph_.reset(new Graph(gridmap_));
  }

  void GridPathPlanner::obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    //ROS_DEBUG("obstacle points is updated");
    ROS_INFO("obstacle points is updated");
    obstacle_points_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *obstacle_points_);
    obstacle_points_frame_id_ = msg->header.frame_id; // check frame_id
  }

  void GridPathPlanner::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    //ROS_DEBUG("pointcloud points is updated");
    ROS_INFO("pointcloud points is updated");
    plane_points_.reset(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(*msg, *plane_points_);
    plane_points_frame_id_ = msg->header.frame_id; // check frame_id
  }

  bool GridPathPlanner::collisionBoundingBoxInfoService(
      jsk_footstep_planner::CollisionBoundingBoxInfo::Request& req,
      jsk_footstep_planner::CollisionBoundingBoxInfo::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    res.box_dimensions.x = collision_bbox_size_[0];
    res.box_dimensions.y = collision_bbox_size_[1];
    res.box_dimensions.z = collision_bbox_size_[2];
    tf::poseEigenToMsg(collision_bbox_offset_, res.box_offset);
    return true;
  }

  void GridPathPlanner::publishText(ros::Publisher& pub,
                                    const std::string& text,
                                    GridPlanningStatus status)
  {
    std_msgs::ColorRGBA ok_color;
    ok_color.r = 0.3568627450980392;
    ok_color.g = 0.7529411764705882;
    ok_color.b = 0.8705882352941177;
    ok_color.a = 1.0;
    std_msgs::ColorRGBA warn_color;
    warn_color.r = 0.9411764705882353;
    warn_color.g = 0.6784313725490196;
    warn_color.b = 0.3058823529411765;
    warn_color.a = 1.0;
    std_msgs::ColorRGBA error_color;
    error_color.r = 0.8509803921568627;
    error_color.g = 0.3254901960784314;
    error_color.b = 0.30980392156862746;
    error_color.a = 1.0;
    std_msgs::ColorRGBA color;
    if (status == OK) {
      color = ok_color;
    }
    else if (status == WARNING) {
      color = warn_color;
    }
    else if (status == ERROR) {
      color = error_color;
    }
    jsk_rviz_plugins::OverlayText msg;
    msg.text = text;
    msg.width = 1000;
    msg.height = 1000;
    msg.top = 10;
    msg.left = 10;
    msg.bg_color.a = 0.0;
    msg.fg_color = color;
    msg.text_size = 24;
    pub.publish(msg);
  }

  void GridPathPlanner::planCB(const jsk_footstep_msgs::PlanFootstepsGoal::ConstPtr& goal)
  {
    boost::mutex::scoped_lock lock(mutex_);
    // latest_header_ = goal->goal_footstep.header;
    ROS_INFO("planCB");

    // check frame_id sanity
    std::string goal_frame_id = goal->initial_footstep.header.frame_id;
#if 0
    if (use_plane_points_) {
      // check perception cloud header
      if (goal_frame_id != plane_points_frame_id_) {
        ROS_ERROR("frame_id of goal and pointcloud do not match. goal: %s, pointcloud: %s.",
                      goal_frame_id.c_str(), plane_points_frame_id_.c_str());
        as_.setPreempted();
        return;
      }
    }
    if (use_obstacle_points_) {
      // check perception cloud header
      if (goal_frame_id != obstacle_points_frame_id_) {
        ROS_ERROR("frame_id of goal and obstacle pointcloud do not match. goal: %s, obstacle: %s.",
                      goal_frame_id.c_str(), obstacle_points_frame_id_.c_str());
        as_.setPreempted();
        return;
      }
    }
#endif
    double initx = 0;
    double inity = 0;
    double goalx = 0;
    double goaly = 0;
    for(int i = 0; i < goal->initial_footstep.footsteps.size(); i++) {
      initx += goal->initial_footstep.footsteps[i].pose.position.x;
      inity += goal->initial_footstep.footsteps[i].pose.position.y;
    }
    initx /= goal->initial_footstep.footsteps.size();
    inity /= goal->initial_footstep.footsteps.size();
    for(int i = 0; i < goal->goal_footstep.footsteps.size(); i++) {
      goalx += goal->goal_footstep.footsteps[i].pose.position.x;
      goaly += goal->goal_footstep.footsteps[i].pose.position.y;
    }
    goalx /= goal->initial_footstep.footsteps.size();
    goaly /= goal->initial_footstep.footsteps.size();

    ROS_INFO("start: %f %f, goal %f %f", initx, inity, goalx, goaly);

    ROS_INFO("build graph");
    buildGraph();

    ROS_INFO("solve");
    Eigen::Vector3f startp(initx, inity, 0);
    Eigen::Vector3f goalp(goalx, goaly, 0);
    int sx, sy, gx, gy;
    pointToGrid(startp, sx, sy);
    pointToGrid(goalp, gx, gy);

    if(!gridmap_->inRange(sx, sy)) {
      ROS_ERROR("start is not in range %d %d", sx, sy);
      as_.setPreempted();
      return;
    }

    if(!gridmap_->inRange(gx, gy)) {
      ROS_ERROR("goal is not in range %d %d", gx, gy);
      as_.setPreempted();
      return;
    }

    GridState::Ptr start_state = graph_->getState(sx, sy);
    GridState::Ptr goal_state = graph_->getState(gx, gy);

    if(start_state->getOccupancy() != 0) {
      ROS_ERROR("start state is occupied");
      as_.setPreempted();
      return;
    }
    if(goal_state->getOccupancy() != 0) {
      ROS_ERROR("goal state is occupied");
      as_.setPreempted();
      return;
    }

    graph_->setStartState(start_state);
    graph_->setGoalState(goal_state);

    Solver solver(graph_);
    //solver.setHeuristic(boost::bind(&gridPerceptionHeuristicDistance, _1, _2));
    solver.setHeuristic(boost::bind(&GridPathPlanner::heuristicDistance, this, _1, _2));

    Solver::Path path = solver.solve();
    if(path.size() < 1) {
      ROS_ERROR("Failed to plan path");
      ROS_INFO("pub marker"); //debug
      publishMarker();        //debug
      as_.setPreempted();
      return;
    }
    ROS_INFO("solved path_size: %d", path.size());
    std::vector<Eigen::Vector3f > points;
    for(int i = 0; i < path.size(); i++) {
      //Solver::StatePtr st = path[i]->getState();
      int ix = path[i]->getState()->indexX();
      int iy = path[i]->getState()->indexY();
      Eigen::Vector3f p;
      gridToPoint(ix, iy, p);
      points.push_back(p);
      ROS_INFO("path %d (%f %f) [%d - %d]", i, ix, iy, p[0], p[1]);
    }
    {
      jsk_recognition_utils::PolyLine pl(points);
      visualization_msgs::MarkerArray ma;
      visualization_msgs::Marker m;
      pl.toMarker(m);
      m.header.frame_id = "map";
      m.id = 101;
      m.scale.x = 0.05;
      ma.markers.push_back(m);
      pub_marker_.publish(ma);
    }
    ROS_INFO("pub marker");
    publishMarker();

    {
      jsk_footstep_msgs::PlanFootstepsResult result_;
      result_.result.header = goal->goal_footstep.header;

      std::vector<Eigen::Vector3f > points;
      for(int i = 0; i < path.size(); i++) {
        int ix = path[i]->getState()->indexX();
        int iy = path[i]->getState()->indexY();
        Eigen::Vector3f p;
        gridToPoint(ix, iy, p);
        jsk_footstep_msgs::Footstep fs;
        fs.leg = jsk_footstep_msgs::Footstep::LLEG;
        fs.dimensions.x = map_resolution_;
        fs.dimensions.y = map_resolution_;
        fs.dimensions.z = 0.01;
        fs.pose.orientation.x = 0;
        fs.pose.orientation.y = 0;
        fs.pose.orientation.z = 0;
        fs.pose.orientation.w = 1;
        fs.pose.position.x = p[0];
        fs.pose.position.y = p[1];
        fs.pose.position.z = p[2];
        result_.result.footsteps.push_back(fs);
      }
      as_.setSucceeded(result_);
    }

#if 0
    pcl::PointCloud<pcl::PointXYZRGB> close_list_cloud, open_list_cloud;
    //solver.openListToPointCloud(open_list_cloud);
    //solver.closeListToPointCloud(close_list_cloud);
    publishPointCloud(close_list_cloud, pub_close_list_, goal->goal_footstep.header);
    publishPointCloud(open_list_cloud,  pub_open_list_,  goal->goal_footstep.header);
#endif

#if 0
    publishText(pub_text_,
                (boost::format("Took %f sec\nPerception took %f sec\nPlanning took %f sec\n%lu path\nopen list: %lu\nclose list:%lu")
                 % planning_duration 
                 % graph_->getPerceptionDuration().toSec()
                 % (planning_duration - graph_->getPerceptionDuration().toSec())
                 % path.size()
                 % open_list_cloud.points.size()
                 % close_list_cloud.points.size()).str(),
                OK);
    ROS_INFO_STREAM("use_obstacle_points: " << graph_->useObstaclePoints());
#endif
  }

  void GridPathPlanner::publishPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
    ros::Publisher& pub,
    const std_msgs::Header& header)
  {
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(cloud, ros_cloud);
    ros_cloud.header = header;
    pub.publish(ros_cloud);
  }

  bool GridPathPlanner::updateCost(GridState::Ptr ptr)
  {
    // determine occupancy
    if(use_obstacle_points_)
    {
      Eigen::Vector3f p;
      gridToPoint(ptr->indexX(), ptr->indexY(), p);

      double pos0 = collision_circle_min_height_ + collision_circle_radius_;
      //double pos1 = collision_circle_max_height_ - collision_circle_radius_;
      int size = 0;
      std::vector<double> pos;
      pos.push_back(pos0);

      for(std::vector<double>::iterator it = pos.begin();
          it != pos.end(); it++) {
        std::vector<int> near_indices;
        std::vector<float> distances;
        pcl::PointXYZ center;
        p[2] = *it;
        center.getVector3fMap() = p;

        obstacle_tree_->radiusSearch(center, collision_circle_radius_, near_indices, distances);

        //std::cerr << "oz: " << near_indices.size() << std::endl;
        size += near_indices.size();
      }
      if(size > 1) { // TODO: occupancy_threshold
        ptr->setOccupancy(1);
      } else {
        ptr->setOccupancy(0);
      }
    }

    // determine cost
    if(use_plane_points_)
    {
      Eigen::Vector3f p;
      gridToPoint(ptr->indexX(), ptr->indexY(), p);

      std::vector<int> plane_indices;
      std::vector<float> distances;
      pcl::PointNormal center;
      center.getVector3fMap() = p;
      plane_tree_->radiusSearch(center, map_resolution_ * 1.4, plane_indices, distances);
      //std::cerr << "ps: " << plane_indices.size() << std::endl;
      // TODO: point num / point average / point deviation
      double cost = 0.0;
      if (plane_indices.size() < 50) { // TODO: plane threshold
        ptr->setOccupancy(2 + ptr->getOccupancy());
      }
      ptr->setCost(cost);
    }

    return false;
  }

  void GridPathPlanner::publishMarker()
  {
    visualization_msgs::MarkerArray ma;

    int sx = gridmap_->sizeX();
    int sy = gridmap_->sizeY();

    { // state plane
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.type = visualization_msgs::Marker::CUBE_LIST;
      m.id = 100;
      m.scale.x = map_resolution_;
      m.scale.y = map_resolution_;
      m.scale.z = 0.01;
      m.color.r = 0.0;
      m.color.g = 0.0;
      m.color.b = 1.0;
      m.color.a = 1.0;

      for(int y = 0; y < sy; y++) {
        for(int x = 0; x < sx; x++) {
          GridState::Ptr st = graph_->getState(x, y);
          Eigen::Vector3f p;
          gridToPoint(x, y, p);
          p[2] = -0.005;
          geometry_msgs::Point gp;
          gp.x = p[0];
          gp.y = p[1];
          gp.z = p[2];
          m.points.push_back(gp);

          std_msgs::ColorRGBA col;
          if(st->getOccupancy() == 1) {
            col.r = 0.0;
            col.g = 0.15;
            col.b = 0.0;
            col.a = 1.0;
          } else if(st->getOccupancy() == 2) {
            col.r = 0.0;
            col.g = 0.0;
            col.b = 0.0;
            col.a = 1.0;
          } else if(st->getOccupancy() == 3) {
            col.r = 0.15;
            col.g = 0.0;
            col.b = 0.0;
            col.a = 1.0;
          } else {
            col.r = 0.0;
            col.g = 0.0;
            col.b = 1.0;
            col.a = 1.0;
          }

          m.colors.push_back(col);
        }
      }
      ma.markers.push_back(m);
    }

    pub_marker_.publish(ma);
  }
}
