#include <memory>
#include <simple_localmap_creator/localmap_creator.h>

namespace simple_localmap_creator {
LocalmapCreator::LocalmapCreator(const Param param)
    : param_(param)
{
    for (const auto& laser_name : param_.laser_names) {
        laser_msg_handlers_.emplace_back(new LaserMsgHandler(laser_name, nh_));
    }
    pub_localmap_ = nh_.advertise<nav_msgs::OccupancyGrid>("/localmap", 1);

    gridmap_.header.frame_id = "base_link";
    gridmap_.info.resolution = param_.map_resolution;
    gridmap_.info.width = static_cast<int>(param.map_width / param.map_resolution);
    gridmap_.info.height = static_cast<int>(param.map_height / param.map_resolution);
    gridmap_.info.origin.position.x = static_cast<double>(-param.map_width / 2.0);
    gridmap_.info.origin.position.y = static_cast<double>(-param.map_height / 2.0);
    gridmap_.info.origin.orientation.w = 1.0;
    grid_center_x_ = static_cast<double>(gridmap_.info.width / 2.0);
    grid_center_y_ = static_cast<double>(gridmap_.info.height / 2.0);
}

bool LocalmapCreator::has_all_laser_scans(void)
{
    for (auto& laser_msg_handler : laser_msg_handlers_) {
        if (!laser_msg_handler->has_laser_scan()) {
            return false;
        }
    }
    return true;
}

Obstacle LocalmapCreator::calc_obstacle_coordinate(const Polar single_scan)
{
    const double x = single_scan.range * std::cos(single_scan.angle);
    const double y = single_scan.range * std::sin(single_scan.angle);

    return { x, y };
}

Obstacle LocalmapCreator::transform_obstacle_coordiname(
    const Obstacle obstacle, std::unique_ptr<LaserMsgHandler>& laser_msg_handler)
{
    geometry_msgs::TransformStamped laser_tf = laser_msg_handler->get_laser_tf();
    geometry_msgs::PoseStamped obstacle_pose;
    obstacle_pose.header.stamp = laser_tf.header.stamp;
    obstacle_pose.header.frame_id = laser_tf.header.frame_id;
    obstacle_pose.pose.position.x = obstacle.x;
    obstacle_pose.pose.position.y = obstacle.y;
    obstacle_pose.pose.orientation.w = 0;

    geometry_msgs::PoseStamped obstacle_pose_transformed;
    obstacle_pose_transformed.header.stamp = laser_tf.header.stamp;
    obstacle_pose_transformed.header.frame_id = laser_tf.child_frame_id;

    tf2::doTransform(obstacle_pose, obstacle_pose_transformed, laser_tf);

    return {
        obstacle_pose_transformed.pose.position.x,
        obstacle_pose_transformed.pose.position.y
    };
}

Pixel LocalmapCreator::calc_pixels_in_gridmap(const Obstacle obstacle)
{
    int px_x = obstacle.x / gridmap_.info.resolution;
    int px_y = obstacle.y / gridmap_.info.resolution;
    const int px_r = std::hypot(px_x, px_y);
    const double px_t = std::atan2(px_y, px_x);

    px_x += grid_center_x_;
    px_y += grid_center_y_;

    return { px_x, px_y, px_r, px_t };
}

bool LocalmapCreator::is_valid_index(const int px_x, const int px_y)
{
    const bool is_too_large = px_x >= (int)gridmap_.info.width || px_y >= (int)gridmap_.info.height;
    const bool is_negative = px_x < 0 || px_y < 0;

    if (is_too_large || is_negative) {
        return false;
    }

    return true;
}

void LocalmapCreator::plot_obstacles(const Pixel obstacle_px)
{
    if (is_valid_index(obstacle_px.x, obstacle_px.y)) {
        const int px_obs = obstacle_px.x + obstacle_px.y * gridmap_.info.width;
        gridmap_.data[px_obs] = 100;
    }

    obstacle_directions_.push_back(obstacle_px.theta);
}

void LocalmapCreator::update_map(void)
{
    for (auto& laser_msg_handler : laser_msg_handlers_) {
        std::vector<Polar> scan_data = laser_msg_handler->get_scan_data();
        for (const auto& single_scan : scan_data) {
            Obstacle obstacle = calc_obstacle_coordinate(single_scan);
            obstacle = transform_obstacle_coordiname(obstacle, laser_msg_handler);
            Pixel obstacle_px = calc_pixels_in_gridmap(obstacle);
            plot_obstacles(obstacle_px);
        }
    }
}

void LocalmapCreator::raycast(void)
{
    const int max_range = std::max(gridmap_.info.width, gridmap_.info.height);

    for (const auto& obstacle_direction : obstacle_directions_) {
        bool has_crossed_obstacle = false;
        for (int r = 0; r <= max_range; r++) {
            const int px_x = r * std::cos(obstacle_direction) + grid_center_x_;
            const int px_y = r * std::sin(obstacle_direction) + grid_center_y_;

            if (!is_valid_index(px_x, px_y)) {
                continue;
            }

            const int index = px_x + px_y * gridmap_.info.width;

            if (gridmap_.data[index] == 100) {
                has_crossed_obstacle = true;
                break;
            }
            if (!has_crossed_obstacle) {
                gridmap_.data[index] = 0;
                continue;
            }
        }
    }
}

void LocalmapCreator::process(void)
{
    ros::Rate loop_rate(param_.hz);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (!has_all_laser_scans()) {
            continue;
        }

        gridmap_.data.clear();
        gridmap_.data.assign(gridmap_.info.height * gridmap_.info.width, -1);
        obstacle_directions_.clear();
        update_map();
        raycast();

        gridmap_.header.stamp = ros::Time::now();
        pub_localmap_.publish(gridmap_);
    }
}
} // namespace simple_localmap_creator
