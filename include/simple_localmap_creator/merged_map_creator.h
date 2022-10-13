#ifndef MERGED_MAP_CREATOR_H
#define MERGED_MAP_CREATOR_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <string>

#include <simple_localmap_creator/localmap_creator.h>

double map_resolution_;
double map_size_;
double map_width_;
double map_height_;
double expansion_rate_;

std::string map_frame_;
std::string map_topic1_;
std::string map_topic2_;

ros::Publisher pub_map_;

#endif  // MERGED_MAP_CREATOR_H
