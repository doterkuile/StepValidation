#include "StepInterpolation.h"

StepInterpolation::StepInterpolation(ros::NodeHandle &nh):
  nh_(nh)
{

  pubGoalStep_ = nh_.advertise<visualization_msgs::MarkerArray>("/stepInterpolation/GoalMarker", 20);
  pubInterpolatedSteps_ = nh_.advertise<visualization_msgs::MarkerArray>("/stepInterpolation/InterpolatedMarkers",20);

  this->setupParams();
  this->setStartGoalPose();

  Eigen::Vector3d goalTranslation = goalPose_.translation()- startPose_.translation();
  double goalDistance = goalTranslation.norm();
  int nrOfSteps = ceil(goalDistance/(maxStepSize_/1.5));

  this->interpolate(nrOfSteps);
  this->setMarkerArray();
}


void StepInterpolation::setupParams()
{


  if(nh_.getParam("/step_validation_config", config_))
    {
      ROS_ERROR_STREAM("Could not retrieve configuration files, results might not be valid");
    }

  footSize_ = eVector3(config_["foot"]["size"]["x"],
                       config_["foot"]["size"]["y"],
                       config_["foot"]["size"]["z"]);

  maxStepSize_ = config_["foot"]["max_step"]["x"];
  maxStepSeparation_ = config_["foot"]["max_step"]["y"];

  goalPosition_ = eVector3(config_["interpolation"]["goal_position"]["x"],
                                          config_["interpolation"]["goal_position"]["y"],
                                          config_["interpolation"]["goal_position"]["z"]);

}

void StepInterpolation::setStartGoalPose()
{
    startPose_ = pal::getTransform("odom","base_footprint", ros::Duration(3.0));

    double goalYaw =std::atan((goalPosition_.y()-startPose_.translation().y()/(goalPosition_.x()-startPose_.translation().x())));

    eVector3 goalOrientation = eVector3(0.0,
                                0.0,
                                goalYaw);

    extractRollPitchYaw(startPose_, &startOrientation_.x(),&startOrientation_.y(), &startOrientation_.z());

    goalPose_  = createMatrix(goalOrientation, goalPosition_);

}

void StepInterpolation::setMarkerArray()
{
  markerArray_.markers.clear();
  for(std::vector<Footstep6D>::iterator step = stepArray_.begin(); step != stepArray_.end(); step++)
  {
    visualization_msgs::Marker marker;
    step->stepToMarker(marker, "/odom", "interpolatedSteps");
    markerArray_.markers.push_back(marker);
  }
  plannerUtils::setMarkerId(markerArray_);
}

void StepInterpolation::interpolate(const int &repeats)
{

  eVector3 startOrientation, goalOrientation;
  extractRollPitchYaw(startPose_, &startOrientation.x(),&startOrientation.y(),&startOrientation.z());
  extractRollPitchYaw(goalPose_, &goalOrientation.x(),&goalOrientation.y(),&goalOrientation.z());


  std::vector<eVector3> rVector = plannerUtils::interpolateRPY(startOrientation, goalOrientation, repeats);
  std::vector<eVector3> tVector = plannerUtils::interpolateTranslation(startPose_.translation(),goalPose_.translation(), repeats);
  pal_locomotion::Side side = pal_locomotion::Side::RIGHT;
  eVector3 sideTranslation;


  for(int ii{0}; ii < rVector.size(); ii++)
  {
        double yaw = rVector[ii].z();

       if(side._value == pal_locomotion::Side::LEFT)
       {
         sideTranslation = eulerRotZ(yaw) * eVector3(0.0, maxStepSeparation_/2, 0.0);
       }
       else
       {
        sideTranslation = eulerRotZ(yaw) * eVector3(0.0, -maxStepSeparation_/2, 0.0);

       }

       tVector[ii] = tVector[ii] + sideTranslation;
       Footstep6D tempstep(tVector[ii],rVector[ii],footSize_, side);
       stepArray_.push_back(tempstep);
       plannerUtils::switchSide(side);

  }

  Footstep6D goalStep(goalPose_.translation(),goalOrientation,footSize_,side);
  stepArray_.push_back(goalStep);
  ROS_INFO_STREAM("Created step array");
  std::cout << "size = " << stepArray_.size() << '\n';

}

void StepInterpolation::publishInterpolationSteps()
{
  pubInterpolatedSteps_.publish(markerArray_);
}

void StepInterpolation::getStepArray(std::vector<Footstep6D> &stepArray)
{
  stepArray = stepArray_;
}

void StepInterpolation::publishGoalPose()
{

  Footstep6D start(startPose_, footSize_);
  Footstep6D goal(goalPose_, footSize_);
  visualization_msgs::Marker startMarker, goalMarker;
  start.stepToMarker(startMarker, "/odom", "interpolatedSteps");
  goal.stepToMarker(goalMarker, "/odom", "interpolatedSteps");
  plannerUtils::setMarkerColor(startMarker,1,1,0);
  plannerUtils::setMarkerColor(goalMarker,1,1,0);
  visualization_msgs::MarkerArray goalMarkers;
  goalMarkers.markers.push_back(startMarker);
  goalMarkers.markers.push_back(goalMarker);
  plannerUtils::setMarkerId(goalMarkers);
  pubGoalStep_.publish(goalMarkers);

}


