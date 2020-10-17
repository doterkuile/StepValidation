#ifndef FOOTWIGGLE_H
#define FOOTWIGGLE_H
#include <grid_map_core/Polygon.hpp>
#include <ros/ros.h>
#include "planner_utils.h"
#include "Footstep6D.h"

class FootWiggle
{
public:
  FootWiggle(Footstep6D &step, const grid_map::Polygon &foot, const grid_map::Polygon &plane, ros::NodeHandle &nh, ros::Publisher &polygonpub);

  void setParameters();
  void wigglePolygon();
  bool footOnPlane(const grid_map::Polygon &polygon);
  bool foundWiggledStep();
  Footstep6D getWiggledStep();

private:

  ros::NodeHandle nh_;
  Footstep6D footStep_;
  Footstep6D wiggledStep_;
  ros::Publisher pubWiggledPolygon_;
  ros::Publisher pubWiggledStep_;
  grid_map::Polygon footPolygon_;
  grid_map::Polygon planePolygon_;
  double wrongDirectionThreshold_;
  double closestDistanceToCliff_;
  double cliffHeightAvoid_;
  double wiggleInsideDelta_;
  Eigen::Vector2d footSize_;
  float wiggleTheta_;
  double wiggleX_;
  bool foundWiggledStep_;



};

#endif // FOOTWIGGLE_H
