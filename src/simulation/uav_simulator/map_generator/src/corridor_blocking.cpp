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

random_device rd;
default_random_engine eng;

ros::Publisher _all_map_pub;
ros::Publisher _all_map_static_obs_pub;
ros::Publisher _all_map_wall_pub;
ros::Publisher _obj_cloud_pub;

double _x_size, _y_size, _z_size;
double _resolution, _pub_rate;
double _corridor_width;
double _blocking_vel;
double _obs_radius;
int _num_blocking_obs;

bool _map_ok = false;

sensor_msgs::PointCloud2 globalMap_pcd;
sensor_msgs::PointCloud2 globalMap_wall_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;
pcl::PointCloud<pcl::PointXYZ> cloudMap_wall;

// Blocking obstacle data
vector<Eigen::Vector2d> obj_pos;
double obj_vel_x;
vector<pcl::PointCloud<pcl::PointXYZ>> obj_clusters;

void ObjUpdate(double dt)
{
  // All blocking obstacles move together at the same velocity
  for (int i = 0; i < _num_blocking_obs; i++)
  {
    // Update position
    obj_pos[i](0) += obj_vel_x * dt;

    // Update point cloud cluster
    for (size_t j = 0; j < obj_clusters[i].points.size(); ++j)
    {
      obj_clusters[i].points[j].x += obj_vel_x * dt;
    }
  }

  // Bounce off corridor ends - all obstacles reverse together
  double x_limit = _x_size / 2.0 - 2.0;
  if (obj_pos[0](0) < -x_limit || obj_pos[_num_blocking_obs - 1](0) > x_limit)
  {
    obj_vel_x = -obj_vel_x;
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

void GenerateBlockingObstacles()
{
  pcl::PointXYZ pt_random;

  double corridor_half = _corridor_width / 2.0;

  // Calculate spacing for 5 obstacles in a row that block the corridor
  // They should be spaced so they completely block passage
  double total_block_width = _num_blocking_obs * (_obs_radius * 2) + (_num_blocking_obs - 1) * 0.1;
  double start_y = -total_block_width / 2.0 + _obs_radius;

  // Start blocking obstacles at a position in front of the UAV
  double start_x = 5.0;

  for (int i = 0; i < _num_blocking_obs; i++)
  {
    pcl::PointCloud<pcl::PointXYZ> cluster;

    double x = start_x;
    double y = start_y + i * (_obs_radius * 2 + 0.1);  // Spaced to block the corridor

    Eigen::Vector2d pos(x, y);
    obj_pos.push_back(pos);

    // Generate point cloud for this obstacle (cylinder)
    int widNum = ceil(_obs_radius / _resolution);
    double h = 2.5; // Obstacle height
    int heiNum = ceil(h / _resolution);

    for (int r = -widNum; r < widNum; r++)
    {
      for (int s = -widNum; s < widNum; s++)
      {
        double temp_x = x + (r + 0.5) * _resolution;
        double temp_y = y + (s + 0.5) * _resolution;

        if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= _obs_radius)
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

  // Set initial velocity (all move together at 0.6 m/s as per paper)
  obj_vel_x = _blocking_vel;

  ROS_WARN("Generated %d blocking obstacles in a row", _num_blocking_obs);
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

  // End walls
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
  ros::init(argc, argv, "corridor_blocking");
  ros::NodeHandle n("~");

  _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);
  _all_map_static_obs_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/static_obs_cloud", 1);
  _all_map_wall_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/wall_cloud", 1);
  _obj_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/obj_cloud", 1);

  // Load parameters
  n.param("map/x_size", _x_size, 40.0);
  n.param("map/y_size", _y_size, 6.0);
  n.param("map/z_size", _z_size, 3.0);
  n.param("map/resolution", _resolution, 0.2);
  n.param("map/corridor_width", _corridor_width, 3.0);
  n.param("map/blocking_vel", _blocking_vel, 0.6);       // Paper: 0.6 m/s
  n.param("map/obs_radius", _obs_radius, 0.5);           // Paper: 0.5m each
  n.param("map/num_blocking_obs", _num_blocking_obs, 5); // Paper: 5 obstacles
  n.param("pub_rate", _pub_rate, 50.0);

  ros::Duration(0.5).sleep();

  unsigned int seed = 3728542744;
  cout << "seed=" << seed << endl;
  eng.seed(seed);

  CorridorGenerate();
  GenerateBlockingObstacles();

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
