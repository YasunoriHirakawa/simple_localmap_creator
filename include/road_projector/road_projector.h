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

class State
{
public:
    State(float _robot_length);
    enum Point
    {
        X=0,
        Y=1,
        Z=2, //not use
    };
    float robot_length;
    bool is_outside_of_road;
    bool project_virtual_wall;
    std::vector<std::vector<float>> virtual_wall_zone0;
    std::vector<std::vector<float>> virtual_wall_zone1;
    geometry_msgs::Point start_point;
    geometry_msgs::Point end_point;
};

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
    bool set_road(const amsl_navigation_msgs::Road &road);
    std::vector<float> transform_road(const std::vector<float> &road, const geometry_msgs::TransformStamped &transform);
    void project_road_to_map(Gridmap *gridmap);
    bool is_in_virtual_wall_zone(const double x, const double y);
    bool is_in_polygon(const double x, const double y, const std::vector<std::vector<float>> &polygon);
    bool on_node();
    void set_virtual_wall_points(const amsl_navigation_msgs::Road& road, const geometry_msgs::TransformStamped& transform, const std::vector<float>& width_sin, const std::vector<float>& width_cos, const std::vector<float>& thickness_sin, const std::vector<float>& thickness_cos);
    bool adjust_virtual_wall_points(const amsl_navigation_msgs::Road& road);
    bool is_hitting_robot(const std::vector<std::vector<float>>& zone);
    float calc_distance_between_point_and_line(const std::vector<float>& l1, const std::vector<float>& l2, const std::vector<float>& point);
    bool robot_is_side_of_zone0();



    bool have_received_road_;
    bool is_outside_of_road_;
    // std::vector<std::vector<float>> road_;
    Gridmap* gridmap_;
    State* current_state_;
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
    // ros::Timer tmr_listen_tf_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

#endif//ROAD_PROJECTOR_H
