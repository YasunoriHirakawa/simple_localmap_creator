#include <simple_localmap_creator/localmap_creator.h>

namespace simple_localmap_creator {
LaserMsgHandler::LaserMsgHandler(const std::string laser_name, ros::NodeHandle& nh)
    : laser_name_(laser_name)
    , tf_listener_(tf_buffer_)
    , has_laser_scan_(false)
{
    const std::string laser_topic = "/" + laser_name_ + "/scan";

    sub_laser_ = nh.subscribe(
        laser_topic, 1, &LaserMsgHandler::laser_scan_callback_,
        this, ros::TransportHints().reliable().tcpNoDelay());
}

void LaserMsgHandler::laser_scan_callback_(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (msg->ranges.empty()) {
        ROS_WARN_STREAM("Scan data is empty.");
        return;
    }

    has_laser_scan_ = true;
    laser_scan_ = *msg;
}

bool LaserMsgHandler::has_laser_scan(void)
{
    return has_laser_scan_;
}

std::vector<Polar> LaserMsgHandler::get_scan_data(void)
{
    std::vector<Polar> scan_data;
    int n_steps = static_cast<int>(laser_scan_.ranges.size());

    for (int i = 0; i < n_steps; i++) {
        float& range = laser_scan_.ranges[i];
        int index_inc = i;
        int index_dec = i;

        while (isinf(range) || isnan(range)) {
            ++index_inc;
            --index_dec;
            float range_a = (index_inc < n_steps) ? laser_scan_.ranges[index_inc] : laser_scan_.ranges[index_dec];
            float range_b = (index_dec >= 0) ? laser_scan_.ranges[index_dec] : laser_scan_.ranges[index_inc];
            range = (range_a + range_b) * 0.5;
        }
        if (range < laser_scan_.range_min || range > laser_scan_.range_max) {
            range = laser_scan_.range_max;
        }

        double angle = laser_scan_.angle_min + laser_scan_.angle_increment * i;
        angle = std::atan2(std::sin(angle), std::cos(angle));
        scan_data.push_back({ angle, range });
    }

    return scan_data;
}

geometry_msgs::TransformStamped LaserMsgHandler::get_laser_tf(void)
{
    return tf_buffer_.lookupTransform(
        "base_link", laser_name_, ros::Time(0));
}
} // namespace simple_localmap_creator
