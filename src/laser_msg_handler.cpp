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

        if (isnan(range)) {
            for (int j = 1; j < n_steps; j++) {
                int index_a = (i + j < n_steps) ? i + j : n_steps - 1;
                int index_b = (i - j >= 0) ? i - j : 0;
                float range_a = laser_scan_.ranges[index_a];
                float range_b = laser_scan_.ranges[index_b];
                if (!isnan(range_a)) {
                    range = range_a;
                    break;
                }
                if (!isnan(range_b)) {
                    range = range_b;
                    break;
                }
            }
        }
        if (
                isinf(range) ||
                range < laser_scan_.range_min || 
                range > laser_scan_.range_max
           ) {
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
