#include "simple_localmap_creator/localmap_creator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localmap_creator");
    simple_localmap_creator::LocalmapCreator localmap_creator;
    localmap_creator.process();

    return 0;
}
