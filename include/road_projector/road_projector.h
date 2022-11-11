#ifndef ROAD_PROJECTOR_H
#define ROAD_PROJECTOR_H

#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>
#include<amsl_navigation_msgs/Road.h>
#include<road_projector/gridmap_editor.h>
#include<tf2/utils.h>
#include<tf2_ros/transform_listener.h>
#include<geometry_msgs/TransformStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseArray.h>


class RoadProjector
{
public:
    RoadProjector();
    ~RoadProjector();
    void process();
private:
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void road_callback(const amsl_navigation_msgs::Road::ConstPtr& msg);
    void publish_projected_map();
    std::vector<std::vector<float>> set_road(const amsl_navigation_msgs::Road &road);
    std::vector<float> transform_road(const std::vector<float> &road, const geometry_msgs::TransformStamped &transform);
    void project_road_to_map(const std::vector<std::vector<float>> &road, Gridmap *gridmap);
    bool is_in_virtual_wall_zone(const double x, const double y, const std::vector<std::vector<float>> &road);
    bool is_in_polygon(const double x, const double y, const std::vector<std::vector<float>> &polygon);
    bool on_node();



    bool have_received_road_=false;
    std::vector<std::vector<float>> road_;
    Gridmap* gridmap_;
    geometry_msgs::PoseArray road_points_;

    bool visualize_=false;
    float road_thickness_;
    float ignore_wall_radius_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_road_;
    ros::Publisher pub_projected_map_;
    ros::Publisher pub_road_points_;
    ros::Publisher pub_node_point_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

#endif//ROAD_PROJECTOR_H
