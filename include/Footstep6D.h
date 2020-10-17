#ifndef Footstep6D_H
#define Footstep6D_H
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math_utils/math_utils.h>
#include "planner_utils.h"
#include <pal_footstep_planner/utils.h>
#include <pal_locomotion/walking_types.h>


class Footstep6D;
typedef std::vector<Footstep6D> FootstepArray;


class Footstep6D
{
public:
  Footstep6D();
  Footstep6D(const Eigen::Vector3d &pos, const Eigen::Quaterniond &q);
  Footstep6D(const Eigen::Vector3d &pos, const Eigen::Vector3d &orientation, const Eigen::Vector3d &size);
  Footstep6D(const Eigen::Vector3d &pos, const Eigen::Quaterniond &q, const Eigen::Vector3d &size);
  Footstep6D(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation, const Eigen::Vector3d &size, const pal_locomotion::Side &side);
  Footstep6D(const Eigen::VectorXd &position, const Eigen::VectorXd &orientation, const eVector3 &size, const pal_locomotion::Side &side);
  Footstep6D(const Footstep6D &step);

  Footstep6D(const Eigen::Isometry3d &pose, const Eigen::Vector3d &size);
  Footstep6D(const Eigen::Isometry3d &pose, const Eigen::Vector3d &size, const pal_locomotion::Side &side);
  void setSide(const pal_locomotion::Side &side);
  void setSize(const Eigen::Vector3d &size);

  void changeSide(const pal_locomotion::Side &side);
  void validateStep(const bool &validStep);
  void setHeight(float &h);
  Eigen::Vector2d get2DPosition() const;
  void stepToMarker(visualization_msgs::Marker &marker, const std::string &headerFrame, const std::string &markerNameSpace) const;
  double getX() const;
  double getY() const;
  double getYaw() const;
  double getPitch() const;
  double getRoll() const;
  Eigen::Vector3d getSize() const;
  pal_locomotion::Side getSide() const;
  bool hasSide() const;
  double getHeight() const;
  eMatrixHom getPose() const;
  void setOrientation(float &roll, float &pitch);
  void setOrientation(const Eigen::VectorXd &rpy);
  void setPosition(const Eigen::VectorXd &position);
  Eigen::Vector3d getRPY() const;
  Eigen::Vector3d getPosition() const;
  bool isValid() const;


private:

  Eigen::Vector3d position_;
  Eigen::Vector3d footSize_;
  bool validStep_;
  Eigen::Vector3d rpy_;
  pal_locomotion::Side side_;
  bool setSide_;

};




#endif // Footstep6D_H
