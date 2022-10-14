#include "simple_localmap_creator/merged_map_creator.h"

void sync_callback(const nav_msgs::OccupancyGrid::ConstPtr& map1, const nav_msgs::OccupancyGrid::ConstPtr& map2)
{

    nav_msgs::OccupancyGrid merged_map;
    merged_map.header = map1->header;
    merged_map.info = map1->info;
    merged_map.info.width = map_width_;
    merged_map.info.height = map_height_;
    merged_map.info.origin.position.x = -map_size_/2.0;
    merged_map.info.origin.position.y = -map_size_/2.0;
    merged_map.info.origin.position.z = 0.0;
    merged_map.info.origin.orientation.w = 1.0;

    merged_map.data.resize(map_width_*map_height_, -1);
    int data_size = map1->data.size();
    for(int i=0; i<data_size; i++)
    {
        merged_map.data[i] = map1->data[i];
        // expand map1 obstacle area : expansion_rate_
        if(map2->data[i] == 100) merged_map.data[i] = 100;
        if(map1->data[i] == 100)
        {
            int x = i%(int)map_width_;
            int y = i/(int)map_width_;
            int expansion = expansion_rate_/map_resolution_;
            for(int j=-expansion; j<=expansion; j++)
            {
                for(int k=-expansion; k<=expansion; k++)
                {
                    int index = (y+j)*map_width_ + (x+k);
                    int index_x = index % (int)map_width_;
                    int index_y = index / (int)map_width_;
                    if(index >= 0 && index < data_size && index_x >= x-expansion && index_x <= x+expansion && index_y >= y-expansion && index_y <= y+expansion)
                    {
                        merged_map.data[index] = 100;
                    }

                }
            }
        }
    }
    pub_map_.publish(merged_map);
    ROS_INFO("Merged map is published");

}

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::OccupancyGrid, nav_msgs::OccupancyGrid> MySyncPolicy;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "merged_map_creator");
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_("~");

    private_nh_.param("map_frame", map_frame_, std::string("base_link"));
    private_nh_.param("map_topic1", map_topic1_, std::string("/velodyne_map"));
    private_nh_.param("map_topic2", map_topic2_, std::string("/hokuyo_map"));
    private_nh_.param("map_size", map_size_, 50.0);
    private_nh_.param("map_resolution", map_resolution_, 0.05);
    private_nh_.param("expansion_rate", expansion_rate_, 0.5);

    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("merged_local_map", 1);

    map_width_ = map_size_ / map_resolution_;
    map_height_ = map_size_ / map_resolution_;

    message_filters::Subscriber<nav_msgs::OccupancyGrid> sub_map1(nh_, map_topic1_, 1);
    message_filters::Subscriber<nav_msgs::OccupancyGrid> sub_map2(nh_, map_topic2_, 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_map1, sub_map2);
    sync.registerCallback(&sync_callback);

    ROS_INFO("Start Merged Map Creator");
    ros::spin();
    return 0;
}
