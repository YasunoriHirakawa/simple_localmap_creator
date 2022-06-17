#ifndef __LOCALMAP_CREATOR_H__
#define __LOCALMAP_CREATOR_H__

#include <cmath>
#include <memory>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

namespace simple_localmap_creator {
struct Param {
    int hz;
    std::vector<std::string> laser_names;
    double map_resolution;
    double map_width;
    double map_height;
};

struct Polar {
    double angle;
    double range;
};

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

class LaserMsgHandler {
public:
    LaserMsgHandler(const std::string laser_name_, ros::NodeHandle& nh);
    bool has_laser_scan(void);
    std::vector<Polar> get_scan_data(void);
    geometry_msgs::TransformStamped get_laser_tf(void);

private:
    void laser_scan_callback_(const sensor_msgs::LaserScan::ConstPtr& msg);

    std::string laser_name_;
    ros::Subscriber sub_laser_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    sensor_msgs::LaserScan laser_scan_;

    bool has_laser_scan_;
    double angle_step_;
    double angle_min_;
};

class LocalmapCreator {
public:
    LocalmapCreator(const Param param);
    bool has_all_laser_scans(void);
    Obstacle calc_obstacle_coordinate(const int i, std::vector<Polar>& scan_data);
    Obstacle transform_obstacle_coordiname(
        const Obstacle obstacle, std::unique_ptr<LaserMsgHandler>& laser_msg_handler);
    Pixel calc_pixels_in_gridmap(const Obstacle obstacle);
    bool is_valid_index(const int px_x, const int px_y);
    void raycast(const Pixel obstacle_px);
    void update_map(void);
    void process(void);

private:
    const Param param_;

    ros::NodeHandle nh_;
    ros::Publisher pub_localmap_;
    std::vector<std::unique_ptr<LaserMsgHandler>> laser_msg_handlers_;

    int map_size_;
    double grid_center_x_;
    double grid_center_y_;

    nav_msgs::OccupancyGrid gridmap_;
};
} // namespace simple_localmap_creator

#endif // __LOCALMAP_CREATOR_H__
