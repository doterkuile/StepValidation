#include "FootWiggle.h"

FootWiggle::FootWiggle(Footstep6D &step, const grid_map::Polygon &foot, const grid_map::Polygon &plane, ros::NodeHandle &nh, ros::Publisher &pubPolygon):
  footPolygon_(foot),
  footStep_(step),
  planePolygon_(plane),
  nh_(nh),
  pubWiggledPolygon_(pubPolygon),
  wiggledStep_(step)
{
  foundWiggledStep_ = false;
  Eigen::Vector3d footSizeTemp = footStep_.getSize();
  footSize_ << footSizeTemp.x(), footSizeTemp.y();
  this->setParameters();
  pubWiggledStep_ = nh_.advertise<visualization_msgs::Marker>("/wiggle/wiggledStep", 20);
  if(!this->footOnPlane(footPolygon_))
  {
    this->wigglePolygon();
    ROS_INFO_STREAM("Foot is not entirely on plane");
  }
}

void FootWiggle::setParameters()
{
  wrongDirectionThreshold_ = 0.04;
  closestDistanceToCliff_ = 0.05;
  cliffHeightAvoid_ = 0.05;
  wiggleInsideDelta_ = 0.03;
  wiggleTheta_ = 5 * M_PI / 180;
  wiggleX_ = 0.03;

}

bool FootWiggle::footOnPlane(const grid_map::Polygon &polygon)
{
  std::vector<grid_map::Position> vertices = polygon.getVertices();
  std::vector<grid_map::Position>::iterator vertex = vertices.begin();
  for( ; vertex != vertices.end(); vertex++)
  {
    if(!planePolygon_.isInside(*vertex))
    {
      return 0;
    }
  }
  return 1;
}

void FootWiggle::wigglePolygon()
{

  std::vector<float> dthetaArray {-wiggleTheta_, 0.0 ,wiggleTheta_};
  std::vector<double> dxArray {-wiggleX_, 0.0, wiggleX_};

  ros::Rate rate(2);
  for(std::vector<float>::iterator dtheta = dthetaArray.begin(); dtheta != dthetaArray.end(); dtheta++)
  {
    for(std::vector<double>::iterator dx = dxArray.begin(); dx !=dxArray.end(); dx++)
    {
      for(std::vector<double>::iterator dy = dxArray.begin(); dy != dxArray.end(); dy++)
      {

        grid_map::Position tWiggle;
        tWiggle << *dx, *dy;
        grid_map::Position newPosition = tWiggle + footStep_.get2DPosition();
        float newYaw = footStep_.getYaw() + *dtheta;
        grid_map::Polygon polygonWiggled = plannerUtils::getStepPolygon(newYaw, newPosition,footSize_);
        polygonWiggled.setFrameId(footPolygon_.getFrameId());
        std::cout << "publishing new polygon" << '\n';
        plannerUtils::publishPolygon("publsih", polygonWiggled,nh_, pubWiggledPolygon_);
//        rate.sleep();
        if(this->footOnPlane(polygonWiggled))
        {
          Eigen::Vector3d stepOrientation;
          stepOrientation << footStep_.getRoll(), footStep_.getPitch(), newYaw;
          Eigen::Vector3d stepPosition;
          stepPosition << newPosition.x(), newPosition.y(), footStep_.getHeight();

          Footstep6D wiggledStep(stepPosition, stepOrientation, footStep_.getSize());
          visualization_msgs::Marker marker;
          wiggledStep.stepToMarker(marker, footPolygon_.getFrameId(), "wiggledStep");
          plannerUtils::setMarkerColor(marker,  1,1,0);
          marker.color.a = 0.5f;
          pubWiggledStep_.publish(marker);
          wiggledStep_ = wiggledStep;
          foundWiggledStep_ = true;

          ROS_INFO_STREAM("Found wiggled step");

          return;
        }

      }
    }
  }
  ROS_INFO_STREAM("Step wiggling not succeeded");

}

bool FootWiggle::foundWiggledStep()
{
  return foundWiggledStep_;
}


Footstep6D FootWiggle::getWiggledStep()
{
  return wiggledStep_;
}
