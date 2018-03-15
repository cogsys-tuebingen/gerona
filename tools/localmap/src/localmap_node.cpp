
#include "localmap.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "LocalMap_Node");
    DE_Localmap demNode;

    ros::Rate r(60);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
