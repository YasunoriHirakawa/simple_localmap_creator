#include "simple_localmap_creator/front_lidar_map.h"

FrontLidarMap::FrontLidarMap():private_nh_("~"),tf_listener_(tf_buffer_)
{
    private_nh_.param("map_frame", map_frame_id_, std::string("base_link"));
    private_nh_.param("map_width", map_width_, 200.0);
    private_nh_.param("map_height", map_height_, 200.0);
    private_nh_.param("map_resolution", map_resolution_, 0.1);
    private_nh_.param("map_origin_x", map_origin_x_, 0.0);
    private_nh_.param("map_origin_y", map_origin_y_, 0.0);
    private_nh_.param("map_origin_yaw", map_origin_yaw_, 0.0);
    private_nh_.param("map_max_range", map_max_range_, 20.0);
    private_nh_.param("map_min_range", map_min_range_, 0.2);
    private_nh_.param("expansion_rate", expansion_rate_, 0.5);

    sub_scan_ = nh_.subscribe("front_hokuyo/scan", 1, &FrontLidarMap::scan_callback, this);
    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/front_lidar_map", 1, true);

    map_size_ = map_width_ * map_resolution_;

    map_.header.frame_id = map_frame_id_;
    map_.info.width = map_width_;
    map_.info.height = map_height_;
    map_.info.resolution = map_resolution_;
    map_.info.origin.position.x = -map_size_/2.0;
    map_.info.origin.position.y = -map_size_/2.0;
    map_.info.origin.position.z = 0.0;
    map_.info.origin.orientation.w = 1;

    xmin_ = -map_size_/2.0;
    xmax_ = map_size_/2.0;
    ymin_ = -map_size_/2.0;
    ymax_ = map_size_/2.0;

    map_max_range_ = 10.0;

}

void FrontLidarMap::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    init_map();
    scan_ = *msg;
    map_.header.stamp = ros::Time::now();
    calc_tf_diff();
    make_map();
    pub_map_.publish(map_);
}

void FrontLidarMap::init_map()
{
    map_.data.resize(map_.info.width * map_.info.height);
    for(int i = 0; i < map_.info.width * map_.info.height; i++) map_.data[i] = -1;
    // ROS_INFO("map initialized");
}

void FrontLidarMap::make_map()
{
    int index;
    double x, y;
    double angle = scan_.angle_min;
    double range_min = scan_.range_min;
    double range_max = scan_.range_max;
    bool is_inrange = true;
    //init smap
    std::vector<int> smap;
    smap.resize(map_.info.width * map_.info.height,-1);
    if(scan_.ranges.size()<=0) return;
    for(auto range : scan_.ranges)
    {
        is_inrange = true;
        if(range <= range_min || range_max <= range)
        {
            is_inrange = false;
            // angle += scan_.angle_increment;
            // continue;
        }

        if(range_min <= range && range < map_min_range_)
        {
            range = map_min_range_;
        }
        else if(range > map_max_range_ ) range = 15.0;
        for (double dist=0;dist<map_size_;dist+=map_resolution_)
        {
            x = dist * cos(angle) + diff_x_;
            y = dist * sin(angle) + diff_y_;
            if(x <= xmin_ || x >= xmax_ || y <= ymin_ || y >= ymax_) continue;
            index = map_index(x, y);
            if(dist >= range && is_inrange)
            {
                smap[index] = 100;
                break;
            }
            else smap[index] = 0;
        }
        angle += scan_.angle_increment;
    }
    //make real map (-y)
    for(int i=0; i < map_width_; i++)
    {
        for(int j=0; j<map_height_; j++)
        {
            map_.data[j+i*map_height_] = smap[j+(map_height_-i-1)*map_width_];
        }
    }

    //if want to expand
    expand_map(map_, expansion_rate_);
    // ROS_INFO("map created");
}

int FrontLidarMap::map_index(double x, double y)
{
    int map_x = (x - map_.info.origin.position.x) / map_.info.resolution;
    int map_y = (y - map_.info.origin.position.y) / map_.info.resolution;
    return map_y * map_.info.width + map_x;
}

void FrontLidarMap::calc_tf_diff()
{
    // get only 2D tf
    get_laser_tf_ = tf_buffer_.lookupTransform("base_link", "front_hokuyo", ros::Time(0));
    diff_x_ = get_laser_tf_.transform.translation.x;
    diff_y_ = get_laser_tf_.transform.translation.y;
    // ROS_INFO("diff_x: %f, diff_y: %f", diff_x_, diff_y_);
}

void FrontLidarMap::expand_map(nav_msgs::OccupancyGrid map, double rate)
{
    if(rate <= 0.0) return;
    nav_msgs::OccupancyGrid exmap = map;
    int map_size = map.data.size();
    int map_width = map.info.width;
    int expansion = rate / map_resolution_;

    for(int i=0; i < map_size; i++)
    {
        if(map.data[i] == 100)
        {
            int x = i % map_width;
            int y = i / map_width;
            for(int j=-expansion; j<=expansion; j++)
            {
                for(int k=-expansion; k<=expansion; k++)
                {
                    int index = (y+j)*map_width + (x+k);
                    if(index >= 0 && index < map_size)
                    {
                        exmap.data[index] = 100;
                    }
                }
            }
        }
    }
    map_ = exmap;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "front_lidar_map");
    FrontLidarMap flm;
    ros::spin();
    return 0;
}

