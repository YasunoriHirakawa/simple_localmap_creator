#ifndef GRIDMAP_EDITOR_H
#define GRIDMAP_EDITOR_H

#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>

class Gridmap
{
    public:
        bool set_map(const nav_msgs::OccupancyGrid &map);
        int get_grid(double y, double x);
        int get_grid(int y, int x);
        double get_resolution(){return this->resolution_;}
        int get_width(){return this->width_;}
        int get_height(){return this->height_;}
        double get_origin_x(){return -1 * this->origin_x_;}
        double get_origin_y(){return -1 * this->origin_y_;}
        std::string get_frame_id(){return this->frame_id_;}
        nav_msgs::OccupancyGrid get_grid();
        bool is_in_map(double y, double x);
        bool set_grid(double y, double x, const int value);
        bool is_in_map(int y, int x);
        bool set_grid(int y, int x, const int value);
        void clear_map(int value);


    private:
        std::vector<std::vector<int>> grid_;
        int width_;
        int height_;
        double origin_x_;
        double origin_y_;
        double resolution_;
        std::string frame_id_;
};

#endif//GRIDMAP_EDITOR_H
