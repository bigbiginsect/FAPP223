#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <chrono>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

using namespace std;

pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
default_random_engine eng;

ros::Publisher _all_map_pub;
ros::Publisher _all_map_static_obs_pub;
ros::Publisher _all_map_wall_pub;
ros::Publisher _obj_cloud_pub;

int _moving_obs_num;
double _x_size, _y_size, _z_size;
double _resolution, _pub_rate;
double _corridor_width;
double _max_vel_x;
double _obs_radius_l, _obs_radius_h;

bool _map_ok = false;

sensor_msgs::PointCloud2 globalMap_pcd;
sensor_msgs::PointCloud2 globalMap_wall_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;
pcl::PointCloud<pcl::PointXYZ> cloudMap_wall;

// Moving obstacle data
vector<Eigen::Vector2d> obj_pos;
vector<Eigen::Vector2d> obj_vel;
vector<double> obj_radius;
vector<pcl::PointCloud<pcl::PointXYZ>> obj_clusters;

uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_vel_x;
uniform_real_distribution<double> rand_radius;

void ObjUpdate(double dt)
{
  for (int i = 0; i < _moving_obs_num; i++)
  {
    // Update position
    obj_pos[i](0) += obj_vel[i](0) * dt;
    obj_pos[i](1) += obj_vel[i](1) * dt;

    // Update point cloud cluster
    for (size_t j = 0; j < obj_clusters[i].points.size(); ++j)
    {
      obj_clusters[i].points[j].x += obj_vel[i](0) * dt;
      obj_clusters[i].points[j].y += obj_vel[i](1) * dt;
    }

    // Bounce off corridor ends
    double x_limit = _x_size / 2.0 - 2.0;
    if (obj_pos[i](0) < -x_limit || obj_pos[i](0) > x_limit)
    {
      obj_vel[i](0) = -obj_vel[i](0);
    }
  }

  // Publish combined moving obstacles
  pcl::PointCloud<pcl::PointXYZ> obj_points;
  for (int i = 0; i < (int)obj_clusters.size(); i++)
  {
    obj_points += obj_clusters[i];
  }

  sensor_msgs::PointCloud2 objCloud_pcd;
  pcl::toROSMsg(obj_points, objCloud_pcd);
  objCloud_pcd.header.frame_id = "world";
  objCloud_pcd.header.stamp = ros::Time::now();
  _obj_cloud_pub.publish(objCloud_pcd);
}

void GenerateMovingObstacles()
{
  pcl::PointXYZ pt_random;

  rand_x = uniform_real_distribution<double>(-_x_size / 2.0 + 5.0, _x_size / 2.0 - 5.0);
  rand_vel_x = uniform_real_distribution<double>(0.5, _max_vel_x);
  rand_radius = uniform_real_distribution<double>(_obs_radius_l, _obs_radius_h);

  double corridor_half = _corridor_width / 2.0;

  for (int i = 0; i < _moving_obs_num; i++)
  {
    pcl::PointCloud<pcl::PointXYZ> cluster;

    double x = rand_x(eng);
    // Randomly place obstacles in either half of corridor
    uniform_real_distribution<double> rand_y(-corridor_half + 0.5, corridor_half - 0.5);
    double y = rand_y(eng);
    double radius = rand_radius(eng);
    double vel_x = rand_vel_x(eng);

    // Half move in positive direction, half in negative
    if (i % 2 == 1)
    {
      vel_x = -vel_x;
    }

    Eigen::Vector2d pos(x, y);
    Eigen::Vector2d vel(vel_x, 0.0);  // No y velocity in corridor

    obj_pos.push_back(pos);
    obj_vel.push_back(vel);
    obj_radius.push_back(radius);

    // Generate point cloud for this obstacle (cylinder)
    int widNum = ceil(radius / _resolution);
    double h = 2.5; // Obstacle height
    int heiNum = ceil(h / _resolution);

    for (int r = -widNum; r < widNum; r++)
    {
      for (int s = -widNum; s < widNum; s++)
      {
        double temp_x = x + (r + 0.5) * _resolution;
        double temp_y = y + (s + 0.5) * _resolution;

        if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= radius)
        {
          for (int t = -5; t < heiNum; t++)
          {
            double temp_z = (t + 0.5) * _resolution;
            pt_random.x = temp_x;
            pt_random.y = temp_y;
            pt_random.z = temp_z;
            cluster.push_back(pt_random);
          }
        }
      }
    }

    obj_clusters.push_back(cluster);
  }
}

void CorridorGenerate()
{
  pcl::PointXYZ pt_random;
  double wall_resolution = _resolution * 2;
  double corridor_half = _corridor_width / 2.0;

  // Generate corridor walls (two sides)
  // Wall at y = -corridor_half
  for (double x = -_x_size / 2.0; x <= _x_size / 2.0; x += wall_resolution)
  {
    for (double z = -1.0; z <= _z_size; z += wall_resolution)
    {
      pt_random.x = x;
      pt_random.y = -corridor_half;
      pt_random.z = z;
      cloudMap.points.push_back(pt_random);
      cloudMap_wall.push_back(pt_random);
    }
  }

  // Wall at y = +corridor_half
  for (double x = -_x_size / 2.0; x <= _x_size / 2.0; x += wall_resolution)
  {
    for (double z = -1.0; z <= _z_size; z += wall_resolution)
    {
      pt_random.x = x;
      pt_random.y = corridor_half;
      pt_random.z = z;
      cloudMap.points.push_back(pt_random);
      cloudMap_wall.push_back(pt_random);
    }
  }

  // End walls (optional, to close the corridor)
  for (double y = -corridor_half; y <= corridor_half; y += wall_resolution)
  {
    for (double z = -1.0; z <= _z_size; z += wall_resolution)
    {
      pt_random.x = -_x_size / 2.0;
      pt_random.y = y;
      pt_random.z = z;
      cloudMap.points.push_back(pt_random);
      cloudMap_wall.push_back(pt_random);

      pt_random.x = _x_size / 2.0;
      cloudMap.points.push_back(pt_random);
      cloudMap_wall.push_back(pt_random);
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  cloudMap_wall.width = cloudMap_wall.points.size();
  cloudMap_wall.height = 1;
  cloudMap_wall.is_dense = true;

  _map_ok = true;

  ROS_WARN("Finished generating corridor map with %lu points", cloudMap.points.size());
}

void pubPoints()
{
  while (ros::ok())
  {
    ros::spinOnce();
    if (_map_ok)
      break;
  }

  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
  _all_map_pub.publish(globalMap_pcd);

  pcl::toROSMsg(cloudMap_wall, globalMap_wall_pcd);
  globalMap_wall_pcd.header.frame_id = "world";
  _all_map_wall_pub.publish(globalMap_wall_pcd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "corridor_bidirectional");
  ros::NodeHandle n("~");

  _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);
  _all_map_static_obs_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/static_obs_cloud", 1);
  _all_map_wall_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/wall_cloud", 1);
  _obj_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/obj_cloud", 1);

  // Load parameters
  n.param("map/x_size", _x_size, 50.0);       // Corridor length (paper: 40m + buffer)
  n.param("map/y_size", _y_size, 6.0);        // Not used directly, for reference
  n.param("map/z_size", _z_size, 3.0);
  n.param("map/resolution", _resolution, 0.2);
  n.param("map/moving_obs_num", _moving_obs_num, 30);
  n.param("map/corridor_width", _corridor_width, 3.0);  // Paper: 3m width
  n.param("map/max_vel_x", _max_vel_x, 5.0);            // Paper: max 5.0 m/s
  n.param("map/obs_radius_l", _obs_radius_l, 0.2);      // Paper: 0.2-0.5m
  n.param("map/obs_radius_h", _obs_radius_h, 0.5);
  n.param("pub_rate", _pub_rate, 50.0);

  ros::Duration(0.5).sleep();

  unsigned int seed = 3728542744;
  cout << "seed=" << seed << endl;
  eng.seed(seed);

  CorridorGenerate();
  GenerateMovingObstacles();

  ros::Rate loop_rate(_pub_rate);
  double dt = 1.0 / _pub_rate;

  while (ros::ok())
  {
    ObjUpdate(dt);
    pubPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
