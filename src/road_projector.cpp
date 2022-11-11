#include<road_projector/road_projector.h>

RoadProjector::RoadProjector(): nh_(""), tf_listener_(tf_buffer_)
{
    sub_map_ = nh_.subscribe("/localmap", 1, &RoadProjector::map_callback, this, ros::TransportHints().tcpNoDelay());
    sub_road_ = nh_.subscribe("/road", 1, &RoadProjector::road_callback, this, ros::TransportHints().tcpNoDelay());
    pub_projected_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/projected_map", 1);

    gridmap_ = new Gridmap();

    ros::NodeHandle private_nh("~");
    private_nh.param<bool>("visualize", visualize_, false);
    private_nh.param<float>("road_thickness", road_thickness_, {0.2});
    if(visualize_) pub_road_points_ = nh_.advertise<geometry_msgs::PoseArray>("/road_points", 1);

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
    nav_msgs::OccupancyGrid map = *msg;
    if(gridmap_->set_map(map))
    {
        // project road zone to map
        project_road_to_map(road_, gridmap_);
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
    road_ = set_road(*msg);
    if (!road_.size()) return;
    have_received_road_ = true;
}

void RoadProjector::project_road_to_map(const std::vector<std::vector<float>> &road, Gridmap *gridmap)
{
    double max_x = gridmap->get_width()*gridmap->get_resolution() + gridmap->get_origin_x();
    double min_x = gridmap->get_origin_x();
    double max_y = gridmap->get_height()*gridmap->get_resolution() + gridmap->get_origin_y();
    double min_y = gridmap->get_origin_y();
    for(double x = min_x; x < max_x; x += gridmap->get_resolution())
    {
        for(double y = min_y; y < max_y; y += gridmap->get_resolution())
        {
            if(is_in_virtual_wall_zone(x, y, road)) gridmap->set_grid(y, x, 100);
        }
    }
}

bool RoadProjector::is_in_virtual_wall_zone(const double x, const double y, const std::vector<std::vector<float>> &road)
{
    std::vector<std::vector<float>> polygon0 = {road[0], road[1], road[2], road[3]};
    std::vector<std::vector<float>> polygon1 = {road[4], road[5], road[6], road[7]};

    if(is_in_polygon(x, y, polygon0) || is_in_polygon(x, y, polygon1)) return true;
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

std::vector<std::vector<float>> RoadProjector::set_road(const amsl_navigation_msgs::Road &road)
{
    std::vector<std::vector<float>> road_points;
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tf_buffer_.lookupTransform("base_link", "map", ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        road_points = road_;
        return road_points;
    }
    float width_sin = 0.5*road.width*std::sin(road.direction);
    float width_cos = 0.5*road.width*std::cos(road.direction);
    float thickness_sin = (road_thickness_ + 0.5*road.width)*std::sin(road.direction);
    float thickness_cos = (road_thickness_ + 0.5*road.width)*std::cos(road.direction);
    // first line points
    road_points.push_back({road.point0.x - width_sin, road.point0.y + width_cos});
    road_points.push_back({road.point0.x - thickness_sin, road.point0.y + thickness_cos});
    road_points.push_back({road.point1.x - thickness_sin, road.point1.y + thickness_cos});
    road_points.push_back({road.point1.x - width_sin, road.point1.y + width_cos});
    // second line points
    road_points.push_back({road.point0.x + width_sin, road.point0.y - width_cos});
    road_points.push_back({road.point0.x + thickness_sin, road.point0.y - thickness_cos});
    road_points.push_back({road.point1.x + thickness_sin, road.point1.y - thickness_cos});
    road_points.push_back({road.point1.x + width_sin, road.point1.y - width_cos});
    if(visualize_)
    {
        road_points_.poses.clear();
        road_points_.header.frame_id = "base_link";
    }
    for(int i=0; i<road_points.size(); i++) road_points[i] = transform_road(road_points[i], transform);
    if(visualize_) pub_road_points_.publish(road_points_);
    return road_points;
}

std::vector<float> RoadProjector::transform_road(const std::vector<float> &road, const geometry_msgs::TransformStamped &transform)
{
    geometry_msgs::Pose pose;
    pose.position.x = road[0];
    pose.position.y = road[1];
    pose.position.z = 0;
    pose.orientation.w = 1.0;
    tf2::doTransform(pose, pose, transform);
    if(visualize_) road_points_.poses.push_back(pose);
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
