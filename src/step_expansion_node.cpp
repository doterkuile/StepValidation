#include "StepValidation.h"
#include <ros/ros.h>


void createStepExpansion(ros::NodeHandle &nh, FootstepArray &stepArray, const grid_map::GridMap &gridMap)
{


    if(!gridMap.exists("elevation"))
    {
        return;
    }


    bool fullMapExpansion;
    nh.param("/full_map_expansion", fullMapExpansion, false);
    eVector2 position, length;// = eVector2(config_["expansion"]["position"]["x"],
    XmlRpc::XmlRpcValue config_;
    nh.getParam("/step_validation_config", config_);

    if(fullMapExpansion)
    {
        position = eVector2(0.0,0.0);
        length = 0.95 * gridMap.getLength();
    }
    else
    {

    position.x() = config_["expansion"]["position"]["x"];
    position.y() = config_["expansion"]["position"]["y"];

    length.x() = config_["expansion"]["length"]["x"];
    length.y() = config_["expansion"]["length"]["y"];

    }

//    grid_map::Position length = eVector2(config_["expansion"]["length"]["x"],
//                                         config_["expansion"]["length"]["y"]);

    double stepYaw = config_["expansion"]["yaw"];
    stepArray.clear();


    bool isSucces;
//    grid_map::SubmapGeometry submap(gridMap,position, length, isSucces);
    grid_map::GridMap submap = gridMap.getSubmap(position,length, isSucces);
    if(!isSucces)
    {
        return;
    }

    for (grid_map::GridMapIterator iterator(submap); !iterator.isPastEnd(); ++iterator)
    {

        eVector3 size(0.2, 0.1, 0.05);
        grid_map::Index index = *iterator;
        eVector3 position;
        eVector2 position2D;
        if(!submap.getPosition(index,position2D))
            continue;
        position = eVector3(position2D.x(), position2D.y(), 0.0);
        eVector3 orientation(0.0,0.0,stepYaw);

        Footstep6D step(position,orientation,size);
        stepArray.push_back(step);
    }



}




void setToMarkerArray(FootstepArray &stepArray, visualization_msgs::MarkerArray &markerArray)
{

    for(FootstepArray::iterator step = stepArray.begin(); step != stepArray.end(); step++)
    {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.ns = "node_expansion";
    marker.header.frame_id = "/odom";

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.2 * step->getSize().x();
    marker.scale.y = 0.2 * step->getSize().y();
    marker.scale.z = 0.2 * step->getSize().z();
    marker.color.a = 1.0f;

    Eigen::Quaterniond q = quaternionRollPitchYaw(step->getRPY());
//    q.setIdentity();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

   marker.pose.position.x = step->getPosition().x();
   marker.pose.position.y = step->getPosition().y();
   marker.pose.position.z = step->getPosition().z();

   if(!step->isValid())
   {
       plannerUtils::setMarkerColor(marker, 1.0, 0.0, 0.0);
   }else
   {
       plannerUtils::setMarkerColor(marker, 0.0, 1.0, 0.0);
   }

   markerArray.markers.push_back(marker);
    }

    plannerUtils::setMarkerId(markerArray);


}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "step_expansion_node");
  ros::NodeHandle nh("~");
  ros::Publisher pubMarker = nh.advertise<visualization_msgs::Marker>("/stepExpansion/expansionList", 1);
  ros::Publisher pubMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/stepExpansion/expansionArray", 1);

  boost::mutex mutex;

  ROS_INFO_STREAM(">>> Starting node");

  StepValidation stepValidator(nh);

  ros::AsyncSpinner spinner(0);

  spinner.start();
  FootstepArray validatedSteps;
  FootstepArray stepArray;


  while(ros::ok())
  {
     boost::lock_guard<boost::mutex> guard(mutex);
     boost::shared_ptr<grid_map::GridMap> map = stepValidator.getGridMap();
     if(map)
     {

     createStepExpansion(nh, stepArray, *map);
     stepValidator.setStepArray(stepArray);
     validatedSteps = stepValidator.getValidatedSteps(stepArray, false);
     visualization_msgs::Marker marker;
     visualization_msgs::MarkerArray markerArray;
     setToMarkerArray(validatedSteps, markerArray);

     stepValidator.stepArrayToMarkerList(validatedSteps, marker, map->getFrameId());

     pubMarker.publish(marker);
     pubMarkerArray.publish(markerArray);
     }

    ros::spinOnce();
  }

  ros::waitForShutdown();


  return 0;
}
