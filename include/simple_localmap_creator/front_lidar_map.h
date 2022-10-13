#ifndef FRONT_LIDAR_MAP_H
#define FRONT_LIDAR_MAP_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <string>

class FrontLidarMap
{
    public:
        FrontLidarMap();

    private:
        void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
        void make_map();
        void init_map();
        int map_index(double x, double y);
        void calc_tf_diff();
        void expand_map(nav_msgs::OccupancyGrid map,double rate);

        //map
        std::string map_frame_id_;
        double map_size_;
        double map_width_;
        double map_height_;
        double xmin_;
        double xmax_;
        double ymin_;
        double ymax_;
        double map_resolution_;
        double map_origin_x_;
        double map_origin_y_;
        double map_origin_yaw_;
        double map_max_range_;
        double map_min_range_;
        double expansion_rate_;

        //tf?
        double diff_x_;
        double diff_y_;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber sub_scan_;
        ros::Publisher pub_map_;

        sensor_msgs::LaserScan scan_;
        nav_msgs::OccupancyGrid map_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        geometry_msgs::TransformStamped get_laser_tf_;
};

#endif // FRONT_LIDAR_MAP_H
