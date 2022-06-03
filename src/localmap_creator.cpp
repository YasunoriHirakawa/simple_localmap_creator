#include "simple_localmap_creator/localmap_creator.h"
#include "sensor_msgs/LaserScan.h"
#include <algorithm>
#include <cmath>

namespace simple_localmap_creator {
LocalmapCreator::LocalmapCreator()
    : nh_private_("~")
{
    nh_private_.param("hz", hz_, { 10 });
    nh_private_.param("map_resolution", map_resolution_, { 0.05 });
    nh_private_.param("map_width", map_width_, { 10.0 });
    nh_private_.param("map_height", map_height_, { 10.0 });

    sub_laser_ = nh_.subscribe(
        "/scan", 1, &LocalmapCreator::laser_callback_, this, ros::TransportHints().reliable().tcpNoDelay());
    pub_localmap_ = nh_.advertise<nav_msgs::OccupancyGrid>("/localmap", 1);

    gridmap_.header.frame_id = "base_link";
    gridmap_.info.resolution = map_resolution_;
    gridmap_.info.width = (int)(map_width_ / map_resolution_);
    gridmap_.info.height = (int)(map_height_ / map_resolution_);
    gridmap_.info.origin.position.x = -(double)(map_width_ / 2.0);
    gridmap_.info.origin.position.y = -(double)(map_height_ / 2.0);
    gridmap_.info.origin.orientation.x = 0.0;
    gridmap_.info.origin.orientation.y = 0.0;
    gridmap_.info.origin.orientation.z = 0.0;
    gridmap_.info.origin.orientation.w = 1.0;
    map_size_ = gridmap_.info.height * gridmap_.info.width;
    grid_center_x_ = (double)(gridmap_.info.width / 2.0);
    grid_center_y_ = (double)(gridmap_.info.height / 2.0);
}

void LocalmapCreator::laser_callback_(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (msg->ranges.empty()) {
        ROS_WARN_STREAM("Scan data is empty.");
        return;
    }

    laser_scan_msg_ = *msg;
}

void LocalmapCreator::update_scan_data_(void)
{
    laser_scan_ = laser_scan_msg_;
}

Obstacle LocalmapCreator::calc_obstacle_coordinate_(int i, double angle)
{
    if (isinf(laser_scan_.ranges[i])) {
        laser_scan_.ranges[i] = std::max(map_height_, map_width_) + 1.0;
    }

    double x = laser_scan_.ranges[i] * std::cos(angle);
    double y = laser_scan_.ranges[i] * std::sin(angle);

    return { x, y, angle };
}

Pixel LocalmapCreator::calc_pixels_in_gridmap_(const Obstacle& obstacle)
{
    int px_x = obstacle.x / gridmap_.info.resolution;
    int px_y = obstacle.y / gridmap_.info.resolution;
    int px_r = std::hypot(px_x, px_y);
    double px_t = obstacle.theta;

    px_x += grid_center_x_;
    px_y += grid_center_y_;

    return { px_x, px_y, px_r, px_t };
}

bool LocalmapCreator::is_valid_index_(int px_x, int px_y)
{
    bool is_too_large = px_x >= (int)gridmap_.info.width || px_y >= (int)gridmap_.info.height;
    bool is_negative = px_x < 0 || px_y < 0;

    if (is_too_large || is_negative) {
        return false;
    }

    return true;
}

void LocalmapCreator::raycast_(Pixel obstacle_px)
{
    for (int r = 0; r <= obstacle_px.range; r++) {
        int px = r * std::cos(obstacle_px.theta) + grid_center_x_;
        int py = r * std::sin(obstacle_px.theta) + grid_center_y_;
        if (!is_valid_index_(px, py)){
            continue;
        }

        int index = py * gridmap_.info.width + px;
        if (r < obstacle_px.range) {
            gridmap_.data[index] = 0;
        } else {
            gridmap_.data[index] = 100;
        }
    }
}

void LocalmapCreator::update_map()
{
    gridmap_.data.clear();
    gridmap_.data.assign(gridmap_.info.height * gridmap_.info.width, -1);

    const double angle_step = laser_scan_.angle_increment;
    const double start_angle = laser_scan_.angle_min;

    for (int i = 0; i < (int)laser_scan_.ranges.size(); i++) {
        double angle = start_angle + angle_step * i;
        angle = std::atan2(std::sin(angle), std::cos(angle));

        Obstacle obstacle = calc_obstacle_coordinate_(i, angle);
        Pixel obstacle_px = calc_pixels_in_gridmap_(obstacle);
        raycast_(obstacle_px);
    }

    gridmap_.header.stamp = ros::Time::now();
}

void LocalmapCreator::process(void)
{
    ros::Rate loop_rate(hz_);

    while (ros::ok()) {
        update_scan_data_();
        if (!laser_scan_.ranges.empty()) {
            update_map();
            pub_localmap_.publish(gridmap_);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
} // namespace simple_localmap_creator
