#include<road_projector/road_projector.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "road_projector_node");
    RoadProjector road_projector;
    road_projector.process();
    return 0;
}
