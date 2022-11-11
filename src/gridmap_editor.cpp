#include<road_projector/gridmap_editor.h>

bool Gridmap::set_map(nav_msgs::OccupancyGrid &map)
{
    if(map.data.size() == 0) ROS_INFO("Gridmap: Data size is 0. only setting map_info");
    else if(map.data.size() != map.info.width*map.info.height)
    {
        ROS_WARN("Gridmap: Cannot set map. Data size is not equal to width*height");
        return false;
    }
    this->grid_.clear();
    this->frame_id_ = map.header.frame_id;
    this->width_ = map.info.width;
    this->height_ = map.info.height;
    this->resolution_ = map.info.resolution;
    this->origin_x_ = -1*(map.info.origin.position.x);
    this->origin_y_ = -1*(map.info.origin.position.y);

    int i=0;
    this->grid_.resize(this->height_ , std::vector<int>(this->width_, -1));
    if(map.data.size())
    {
        for(const auto &cell: map.data)
        {
            this->grid_[i/this->width_][(i)%this->width_] = cell;
            i++;
        }
    }
    // ROS_INFO("Gridmap: width is set to %d, height %d", this->width_, this->height_);
    return true;
}

int Gridmap::get_grid(double y, double x)
{
    int grid_y, grid_x;
    if(!is_in_map(y, x)) return -1; //if x or y is out of range return -1
    grid_y = (int)((y+this->origin_y_)/this->resolution_);
    grid_x = (int)((x+this->origin_x_)/this->resolution_);
    // ROS_WARN_STREAM(grid_y<<","<<grid_x);
    return this->grid_[grid_y][grid_x];
}

int Gridmap::get_grid(int y, int x)
{
    if(!is_in_map(y, x)) return -1; //if x or y is out of range return -1
    return this->grid_[y][x];
}

nav_msgs::OccupancyGrid Gridmap::get_grid()
{
    nav_msgs::OccupancyGrid map;
    map.header.frame_id = this->frame_id_;
    map.header.stamp = ros::Time::now();

    map.info.resolution = this->resolution_;
    map.info.width = this->width_;
    map.info.height = this->height_;
    map.info.origin.position.x = -1*this->origin_x_;
    map.info.origin.position.y = -1*this->origin_y_;
    map.info.origin.orientation.w = 1.0;

    for(const auto &row : this->grid_)
    {
        for(const auto &cell : row)
        {
            map.data.push_back(cell);
        }
    }
    return map;
}

bool Gridmap::is_in_map(double y, double x)
{
    if( (y+this->origin_y_) < 0 || (y+this->origin_y_)/this->resolution_ >= this->height_ || (x+this->origin_x_) < 0 || (x+this->origin_x_)/this->resolution_ >= this->width_) return false;
    return true;
}

bool Gridmap::set_grid(double y, double x, const int value)
{
    int grid_y, grid_x;
    if(!is_in_map(y, x)) return false;
    grid_y = (int)((y+this->origin_y_)/this->resolution_);
    grid_x = (int)((x+this->origin_x_)/this->resolution_);
    // ROS_WARN_STREAM(grid_y<<","<<grid_x);
    this->grid_[grid_y][grid_x]= value;
    return true;
}

bool Gridmap::is_in_map(int y, int x)
{
    if(y < 0 || y >= this->height_ || x < 0 || x >= this->width_) return false;
    return true;
}

bool Gridmap::set_grid(int y, int x, const int value)
{
    if(!is_in_map(y, x)) return false;
    this->grid_[y][x] = value;
    return true;
}

void Gridmap::clear_map(int value)
{
    this->grid_.clear();
    this->grid_.resize(this->height_ , std::vector<int>(this->width_, value));
}

