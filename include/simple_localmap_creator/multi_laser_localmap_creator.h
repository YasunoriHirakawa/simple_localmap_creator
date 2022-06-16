#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <simple_localmap_creator/localmap_creator.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

namespace simple_localmap_creator {
class LaserMsgHandler {
public:
    LaserMsgHandler(const int id, ros::NodeHandle& nh);
    sensor_msgs::LaserScan get_laser_scan(void);

private:
    void laser_scan_callback_(sensor_msgs::LaserScan::ConstPtr& msg);

    const int id_;
    ros::Subscriber sub_laser_;
    sensor_msgs::LaserScan laser_scan_;
};

class MultiLaserLocalmapCreator : public LocalmapCreator {
public:
    MultiLaserLocalmapCreator(const int n_laser, const Param param);
    void raycast(const Pixel obstacle_px);

private:
    LaserMsgHandler laser_msg_hundler;
};
} // namespace simple_localmap_creator
