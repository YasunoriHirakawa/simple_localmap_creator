#include <simple_localmap_creator/localmap_creator.h>

int main(int argc, char** argv)
{
    simple_localmap_creator::Param param;

    ros::init(argc, argv, "localmap_creator");
    ros::param::param<int>("~hz", param.hz, 10);
    ros::param::param<std::vector<std::string>>(
        "~laser_names", param.laser_names, std::vector<std::string>());
    ros::param::param<double>("~map_resolution", param.map_resolution, 0.05);
    ros::param::param<double>("~map_width", param.map_width, 10.0);
    ros::param::param<double>("~map_height", param.map_height, 10.0);

    simple_localmap_creator::LocalmapCreator localmap_creator(param);
    localmap_creator.process();

    return 0;
}
