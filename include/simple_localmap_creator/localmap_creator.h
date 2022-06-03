// localmap_creater.h ---HEADER of local map creater---
// 2020.3.31 Kawai Hibiki , Hirakawa Yasunori

#ifndef __LOCALMAP_CREATOR_H__
#define __LOCALMAP_CREATOR_H__

#include <cmath>
#include <geometry_msgs/Point.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

namespace simple_localmap_creator {
struct Obstacle {
    double x;
    double y;
    double theta;
};

struct Pixel {
    int x;
    int y;
    int range;
    double theta;
};

class LocalmapCreator {
public:
    LocalmapCreator();
    void update_map();
    void process();

private:
    void laser_callback_(const sensor_msgs::LaserScan::ConstPtr& msg);
    void update_scan_data_();
    Obstacle calc_obstacle_coordinate_(int i, double angle);
    Pixel calc_pixels_in_gridmap_(const Obstacle& obstacle);
    bool is_valid_index_(int px_x, int px_y);
    void raycast_(Pixel obstacle_px);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher pub_localmap_;
    ros::Subscriber sub_laser_;

    int hz_;
    double map_resolution_;
    double map_width_;
    double map_height_;
    int map_size_;
    double grid_center_x_;
    double grid_center_y_;

    sensor_msgs::LaserScan laser_scan_msg_;
    sensor_msgs::LaserScan laser_scan_;
    nav_msgs::OccupancyGrid gridmap_;

};
} // namespace simple_localmap_creator

#endif // __LOCALMAP_CREATOR_H__
