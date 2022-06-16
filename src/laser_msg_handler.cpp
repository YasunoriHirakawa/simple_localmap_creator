#include <simple_localmap_creator/localmap_creator.h>

namespace simple_localmap_creator {
LaserMsgHandler::LaserMsgHandler(const std::string laser_name, ros::NodeHandle& nh)
    : laser_name_(laser_name)
    , tf_listener_(tf_buffer_)
    , has_laser_scan_(false)
    , angle_step_(0.0)
    , angle_min_(0.0)
{
    const std::string laser_topic = "/" + laser_name_ + "/scan";

    sub_laser_ = nh.subscribe(
        laser_topic, 1, &LaserMsgHandler::laser_scan_callback_,
        this, ros::TransportHints().reliable().tcpNoDelay());
}

void LaserMsgHandler::laser_scan_callback_(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (!has_laser_scan_) {
        has_laser_scan_ = true;
        angle_step_ = msg->angle_increment;
        angle_min_ = msg->angle_min;
    }
    if (msg->ranges.empty()) {
        ROS_WARN_STREAM("Scan data is empty.");
        return;
    }

    laser_scan_ = *msg;
}

bool LaserMsgHandler::has_laser_scan(void)
{
    return has_laser_scan_;
}

std::vector<Polar> LaserMsgHandler::get_scan_data(void)
{
    std::vector<Polar> scan_data;

    for (int i = 0; i < static_cast<int>(laser_scan_.ranges.size()); i++) {
        double angle = angle_min_ + angle_step_ * i;
        angle = std::atan2(std::sin(angle), std::cos(angle));
        scan_data.push_back({ angle, laser_scan_.ranges[i] });
    }

    return scan_data;
}

geometry_msgs::TransformStamped LaserMsgHandler::get_laser_tf(void)
{
    return tf_buffer_.lookupTransform(
        "base_link", laser_name_, ros::Time(0));
}
} // namespace simple_localmap_creator
