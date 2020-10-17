#ifndef STEPINTERPOLATION_H
#define STEPINTERPOLATION_H
#include <ros/ros.h>
#include <Eigen/Dense>
#include <math_utils/math_utils.h>
#include <math_utils/trajectory/quaternion_interp/quaternion_cubic_spline.h>
#include <pal_geometry_tools/wykobi.hpp>
#include "Footstep6D.h"
#include "planner_utils.h"
#include "StepValidation.h"
#include <math_utils/geometry_tools.h>
#include <pal_ros_utils/tf_utils.h>
#include <pal_locomotion/walking_types.h>


class StepInterpolation
{
public:
  StepInterpolation(ros::NodeHandle &nh);
  void setupParams();
  void setStartGoalPose();
  void publishGoalPose();
  void interpolate(const int &repeat);
  void publishInterpolationSteps();
  void getStepArray(std::vector<Footstep6D> &stepArray);
  void setMarkerArray();
  private:

  ros::NodeHandle nh_;
  ros::Publisher pubGoalStep_;
  ros::Publisher pubInterpolatedSteps_;

  XmlRpc::XmlRpcValue config_;

  eMatrixHom goalPose_;
  eVector3 startOrientation_;
  eVector3 goalPosition_;
  eMatrixHom startPose_;
  Eigen::Vector3d footSize_;
  std::vector<Footstep6D> stepArray_;
  visualization_msgs::MarkerArray markerArray_;
  double maxStepSize_;
  double maxStepSeparation_;

};

#endif // STEPINTERPOLATION_H
