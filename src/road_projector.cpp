#include<road_projector/road_projector.h>

State::State(float _robot_length)
{
    robot_length = _robot_length;
    is_outside_of_road = false;
    project_virtual_wall = false;
    std::vector<float> pre_point = {0.0, 0.0};
    std::vector<std::vector<float>> pre_virtual_wall_zone = { pre_point, pre_point, pre_point, pre_point};
    virtual_wall_zone0 = pre_virtual_wall_zone;
    virtual_wall_zone1 = pre_virtual_wall_zone;
    geometry_msgs::Point pre_point_msg;
    pre_point_msg.x = 0.0;
    pre_point_msg.y = 0.0;
    pre_point_msg.z = 0.0;
    start_point = pre_point_msg;
    end_point = pre_point_msg;
}

RoadProjector::RoadProjector(): nh_(""), tf_listener_(tf_buffer_)
{
    sub_map_ = nh_.subscribe("/localmap", 1, &RoadProjector::map_callback, this, ros::TransportHints().tcpNoDelay());
    sub_road_ = nh_.subscribe("/road", 1, &RoadProjector::road_callback, this, ros::TransportHints().tcpNoDelay());
    pub_projected_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/projected_map", 1);
    // tmr_listen_tf_ = nh_.createTimer(ros::Duration(0.05), &RoadProjector::listen_tf, this);

    gridmap_ = new Gridmap();

    ros::NodeHandle private_nh("~");
    private_nh.param<bool>("visualize", visualize_, false);
    private_nh.param<float>("road_thickness", road_thickness_, {0.2});
    private_nh.param<float>("ignore_wall_radius", ignore_wall_radius_, {2.0});
    float robot_length;
    private_nh.param<float>("robot_length", robot_length, {0.5});
    have_received_road_ = false;
    is_outside_of_road_ = false;
    current_state_ = new State(robot_length);
    if(visualize_)
    {
        pub_road_points_ = nh_.advertise<geometry_msgs::PoseArray>("/road_points", 1);
        pub_node_point_ = nh_.advertise<geometry_msgs::PoseStamped>("/node_point", 1);
    }

}

RoadProjector::~RoadProjector(){}

void RoadProjector::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    if(!have_received_road_) return;
    if(msg->data.size() == 0)
    {
        ROS_WARN("mapdata is empty");
        return;
    }
    if(visualize_)
    {
        road_points_.poses.clear();
        road_points_.header.frame_id = "base_link";
        int i = 0;
        for(const auto& point: current_state_->virtual_wall_zone0)
        {
            geometry_msgs::Pose pose;
            pose.position.x = point[State::Point::X];
            pose.position.y = point[State::Point::Y];
            road_points_.poses.push_back(pose);
            i++;
        }
        i = 0;
        for(const auto& point: current_state_->virtual_wall_zone1)
        {
            geometry_msgs::Pose pose;
            pose.position.x = point[State::Point::X];
            pose.position.y = point[State::Point::Y];
            road_points_.poses.push_back(pose);
            i++;
        }
        pub_road_points_.publish(road_points_);
    }
    if(on_node() || !current_state_->project_virtual_wall)
    {
        pub_projected_map_.publish(msg);
        return;
    }
    if(gridmap_->set_map(*msg))
    {
        // project road zone to map
        project_road_to_map(gridmap_);
        publish_projected_map();
    }
    else
    {
        ROS_WARN("failed to set map");
        return;
    }
}

void RoadProjector::road_callback(const amsl_navigation_msgs::Road::ConstPtr& msg)
{
    if(!set_road(*msg)) return;
    have_received_road_ = true;
}

bool RoadProjector::on_node()
{
    float node_x = current_state_->start_point.x;
    float node_y = current_state_->start_point.y;
    if(visualize_)
    {
        geometry_msgs::PoseStamped node_point;
        node_point.header.frame_id = "base_link";
        node_point.header.stamp = ros::Time::now();
        node_point.pose.position.x = node_x;
        node_point.pose.position.y = node_y;
        node_point.pose.orientation.w = 1.0;
        pub_node_point_.publish(node_point);
        // ROS_INFO("dist_from_node: %f", std::hypot(node_x, node_y));
    }

    if(std::hypot(node_x, node_y) < ignore_wall_radius_) return true;
    else return false;
}

void RoadProjector::project_road_to_map(Gridmap *gridmap)
{
    double max_x = gridmap->get_width()*gridmap->get_resolution() + gridmap->get_origin_x();
    double min_x = gridmap->get_origin_x();
    double max_y = gridmap->get_height()*gridmap->get_resolution() + gridmap->get_origin_y();
    double min_y = gridmap->get_origin_y();
    for(double x = min_x; x < max_x; x += gridmap->get_resolution())
    {
        for(double y = min_y; y < max_y; y += gridmap->get_resolution())
        {
            if(is_in_virtual_wall_zone(x, y)) gridmap->set_grid(y, x, 100);
        }
    }
}

bool RoadProjector::is_in_virtual_wall_zone(const double x, const double y)
{
    if(is_in_polygon(x, y, current_state_->virtual_wall_zone0) || is_in_polygon(x, y, current_state_->virtual_wall_zone1)) return true;
    else return false;
}

bool RoadProjector::is_in_polygon(const double x, const double y, const std::vector<std::vector<float>> &polygon)
{
    std::vector<float> obs = {x, y};
    int cn = 0;
    int n = polygon.size();
    for(int i=0; i<n; i++)
    {
        if(  ((polygon[i][1] <= obs[1]) && (polygon[(i+1)%n][1] > obs[1])) ||
             ((polygon[i][1] > obs[1]) && (polygon[(i+1)%n][1] <= obs[1])) )
        {
            float vt = (obs[1] - polygon[i][1]) / (polygon[(i+1)%n][1] - polygon[i][1]);
            if(obs[0] < (polygon[i][0] + (vt * (polygon[(i+1)%n][0] - polygon[i][0]) ) ) )
                cn++;
        }
    }
    if(cn%2 == 0) return false;
    else return true;
}

bool RoadProjector::set_road(const amsl_navigation_msgs::Road &road)
{
    current_state_->start_point = road.point0;
    current_state_->end_point = road.point1;
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tf_buffer_.lookupTransform("base_link", "map", ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return false;
    }
    tf2::doTransform(current_state_->start_point, current_state_->start_point, transform);
    tf2::doTransform(current_state_->end_point, current_state_->end_point, transform);


    //zone0 = right
    float rightside_width = road.distance_to_right;
    float leftside_width = road.width - road.distance_to_right;
    std::vector<float> width_sin = {rightside_width*std::sin(road.direction), leftside_width*std::sin(road.direction)};
    std::vector<float> width_cos = {rightside_width*std::cos(road.direction), leftside_width*std::cos(road.direction)};
    std::vector<float> thickness_sin = {(road_thickness_ + rightside_width)*std::sin(road.direction), (road_thickness_ + leftside_width)*std::sin(road.direction)};
    std::vector<float> thickness_cos = {(road_thickness_ + rightside_width)*std::cos(road.direction), (road_thickness_ + leftside_width)*std::cos(road.direction)};

    if(current_state_->is_outside_of_road)
    {
        std::vector<std::vector<float>> check_zone; //outside of road
        check_zone.push_back({road.point0.x - width_sin[0], road.point0.y + width_cos[0]});
        check_zone.push_back({road.point1.x - width_sin[0], road.point1.y + width_cos[0]});
        check_zone.push_back({road.point1.x + width_sin[1], road.point1.y - width_cos[1]});
        check_zone.push_back({road.point0.x + width_sin[1], road.point0.y - width_cos[1]});
        for(auto& point: check_zone) point = transform_road(point, transform);
        set_virtual_wall_points(road, transform, width_sin, width_cos, thickness_sin, thickness_cos);
        if(is_in_polygon(0.0, 0.0, check_zone))
        {
            current_state_->project_virtual_wall = true;
            current_state_->is_outside_of_road = adjust_virtual_wall_points(road);
            ROS_ERROR_STREAM("outside: " << current_state_->is_outside_of_road);
        }
        else current_state_->project_virtual_wall = false;

    }
    else
    {
        std::vector<std::vector<float>> check_zone; //outside of wall
        check_zone.push_back({road.point0.x - thickness_sin[0], road.point0.y + thickness_cos[0]});
        check_zone.push_back({road.point1.x - thickness_sin[0], road.point1.y + thickness_cos[0]});
        check_zone.push_back({road.point1.x + thickness_sin[1], road.point1.y - thickness_cos[1]});
        check_zone.push_back({road.point0.x + thickness_sin[1], road.point0.y - thickness_cos[1]});
        for(auto& point: check_zone) point = transform_road(point, transform);
        set_virtual_wall_points(road, transform, width_sin, width_cos, thickness_sin, thickness_cos);
        current_state_->is_outside_of_road = !is_in_polygon(0.0, 0.0, check_zone);
        if(current_state_->is_outside_of_road) current_state_->project_virtual_wall = false;
        else current_state_->project_virtual_wall = true;
    }

    return true;
}

bool RoadProjector::adjust_virtual_wall_points(const amsl_navigation_msgs::Road& road)
{
    // if adjusted return true, not need to adjust return false
    if(robot_is_side_of_zone0())
    {
        // ROS_INFO("robot is rightside of road");
        int count = 0;
        float resolution = 0.1;
        int max_count = (int)((road_thickness_ + current_state_->robot_length)/resolution);
        float dx10 = (current_state_->virtual_wall_zone0[1][State::Point::X] - current_state_->virtual_wall_zone0[0][State::Point::X]) / road_thickness_ * resolution;
        float dy10 = (current_state_->virtual_wall_zone0[1][State::Point::Y] - current_state_->virtual_wall_zone0[0][State::Point::Y]) / road_thickness_ * resolution;
        while(is_hitting_robot(current_state_->virtual_wall_zone0))
        {
            current_state_->virtual_wall_zone0[0][State::Point::X] += dx10;
            current_state_->virtual_wall_zone0[0][State::Point::Y] += dy10;
            current_state_->virtual_wall_zone0[1][State::Point::X] += dx10;
            current_state_->virtual_wall_zone0[1][State::Point::Y] += dy10;

            current_state_->virtual_wall_zone0[2][State::Point::X] += dx10;
            current_state_->virtual_wall_zone0[2][State::Point::Y] += dy10;
            current_state_->virtual_wall_zone0[3][State::Point::X] += dx10;
            current_state_->virtual_wall_zone0[3][State::Point::Y] += dy10;

            count++;
            if(count > max_count)
            {
                ROS_WARN("road projector: adjested wall max times %d", count);
                break;
            }
        }
        if(count) return true;
        else return false;
    }
    else
    {
        // ROS_INFO("robot is leftside of road");
        int count = 0;
        float resolution = 0.1;
        int max_count = (int)((road_thickness_ + current_state_->robot_length)/resolution);
        float dx10 = (current_state_->virtual_wall_zone1[1][State::Point::X] - current_state_->virtual_wall_zone1[0][State::Point::X]) / road_thickness_ * resolution;
        float dy10 = (current_state_->virtual_wall_zone1[1][State::Point::Y] - current_state_->virtual_wall_zone1[0][State::Point::Y]) / road_thickness_ * resolution;
        // ROS_INFO_STREAM("dx10: " << dx10 << " dy10: " << dy10);
        // float dx23 = (current_state_->virtual_wall_zone1[3][State::Point::X] - current_state_->virtual_wall_zone1[2][State::Point::X]) / road_thickness_ * resolution;
        // float dy23 = current_state_->virtual_wall_zone1[3][State::Point::Y] - current_state_->virtual_wall_zone1[2][State::Point::Y] / road_thickness_ * resolution;
        while(is_hitting_robot(current_state_->virtual_wall_zone1) )
        {
            current_state_->virtual_wall_zone1[0][State::Point::X] += dx10;
            current_state_->virtual_wall_zone1[0][State::Point::Y] += dy10;
            current_state_->virtual_wall_zone1[1][State::Point::X] += dx10;
            current_state_->virtual_wall_zone1[1][State::Point::Y] += dy10;

            current_state_->virtual_wall_zone1[2][State::Point::X] += dx10;
            current_state_->virtual_wall_zone1[2][State::Point::Y] += dy10;
            current_state_->virtual_wall_zone1[3][State::Point::X] += dx10;
            current_state_->virtual_wall_zone1[3][State::Point::Y] += dy10;

            count++;
            if(count > max_count)
            {
                ROS_WARN("road projector: adjested wall max times %d", count);
                break;
            }
        }
        // ROS_INFO_STREAM("count: " << count);
        if(count) return true;
        else return false;
    }
}

bool RoadProjector::is_hitting_robot(const std::vector<std::vector<float>>& zone)
{
    float wall_distance = calc_distance_between_point_and_line(zone[0], zone[3], {0.0, 0.0});
    // ROS_WARN_STREAM("wall_distance: " << wall_distance << " robot_length: " << current_state_->robot_length);
    if(wall_distance < current_state_->robot_length) return true;
    else return false;
}

float RoadProjector::calc_distance_between_point_and_line(const std::vector<float>& l1, const std::vector<float>& l2, const std::vector<float>& point)
{
    float dx21 = l2[0] - l1[0];
    float dy21 = l2[1] - l1[1];
    float dx10 = l1[0] - point[0];
    float dy10 = l1[1] - point[1];
    float dx20 = l2[0] - point[0];
    float dy20 = l2[1] - point[1];

    double t = -(dx21*dx10 + dy21*dy10);
    if(t < 0) return std::hypot(dx10, dy10);
    else if(t > dx21*dx21 + dy21*dy21) return std::hypot(dx20, dy20);
    else return sqrt((dx21*dy10 - dy21*dx10) * (dx21*dy10 - dy21*dx10) / (dx21*dx21 + dy21*dy21));
}

bool RoadProjector::robot_is_side_of_zone0()
{
    float d0 = std::hypot(current_state_->virtual_wall_zone0[0][State::Point::X], current_state_->virtual_wall_zone0[0][State::Point::Y]);
    float d1 = std::hypot(current_state_->virtual_wall_zone1[0][State::Point::X], current_state_->virtual_wall_zone1[0][State::Point::Y]);
    if(d0 < d1) return true;
    else return false;
}

void RoadProjector::set_virtual_wall_points(const amsl_navigation_msgs::Road& road, const geometry_msgs::TransformStamped& transform, const std::vector<float>& width_sin, const std::vector<float>& width_cos, const std::vector<float>& thickness_sin, const std::vector<float>& thickness_cos)
{
        // first line points
        current_state_->virtual_wall_zone0[0] = {road.point0.x - width_sin[0], road.point0.y + width_cos[0]};
        current_state_->virtual_wall_zone0[1] = {road.point0.x - thickness_sin[0], road.point0.y + thickness_cos[0]};
        current_state_->virtual_wall_zone0[2] = {road.point1.x - thickness_sin[0], road.point1.y + thickness_cos[0]};
        current_state_->virtual_wall_zone0[3] = {road.point1.x - width_sin[0], road.point1.y + width_cos[0]};
        // second line points
        current_state_->virtual_wall_zone1[0] = {road.point0.x + width_sin[1], road.point0.y - width_cos[1]};
        current_state_->virtual_wall_zone1[1] = {road.point0.x + thickness_sin[1], road.point0.y - thickness_cos[1]};
        current_state_->virtual_wall_zone1[2] = {road.point1.x + thickness_sin[1], road.point1.y - thickness_cos[1]};
        current_state_->virtual_wall_zone1[3] = {road.point1.x + width_sin[1], road.point1.y - width_cos[1]};
        for(auto& point: current_state_->virtual_wall_zone0) point = transform_road(point, transform);
        for(auto& point: current_state_->virtual_wall_zone1) point = transform_road(point, transform);
}

std::vector<float> RoadProjector::transform_road(const std::vector<float> &road, const geometry_msgs::TransformStamped &transform)
{
    geometry_msgs::Pose pose;
    pose.position.x = road[0];
    pose.position.y = road[1];
    pose.position.z = 0;
    pose.orientation.w = 1.0;
    tf2::doTransform(pose, pose, transform);
    return {pose.position.x, pose.position.y};
}

void RoadProjector::publish_projected_map()
{
    pub_projected_map_.publish(gridmap_->get_grid());
}

void RoadProjector::process()
{
    ros::spin();
}
