//#include <pal_footstep_planner/foot_step_planning/ObstacleMap.h>
#include "StepValidation.h"


int main(int argc, char **argv)
{


    ros::init(argc, argv, "step_validation_node");
    ros::NodeHandle nh("~");

    ROS_INFO_STREAM(">>> Starting node");
    StepValidation stepValidation(nh);

    while(ros::ok())
    {
      ros::spinOnce();
    }


    return 0;

}
