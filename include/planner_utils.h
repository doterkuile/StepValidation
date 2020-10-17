#ifndef PLANNER_UTILS_H
#define PLANNER_UTILS_H
#include "Footstep6D.h"
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <math_utils/math_utils.h>
#include <pal_locomotion/walking_types.h>
#include <chrono>


namespace plannerUtils
{


void setupMarker(visualization_msgs::Marker &marker, const std::string &headerFrame, const std::string &markerNameSpace);

void setMarkerId(visualization_msgs::MarkerArray &markerArray);

bool transformCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud,  std::shared_ptr<tf2_ros::TransformListener> &tfListener, tf2_ros::Buffer &tfBuffer_);

void getMaxIndex(const Eigen::MatrixXf &m, grid_map::Index &index);

void getNumberOfDifferentValues(const Eigen::MatrixXf &m, int &n);

void switchSide(pal_locomotion::Side &side);

std::vector<eVector3> getStepCorners(const eVector3 &stepPosition, const eVector3 &stepOrientation, const eVector3 &stepSize);

void setRightRotationDirection(eVector3 &rpy);

grid_map::Matrix flipPlane(const eVector3 &rpy, const eVector3 &stepPosition, const grid_map::GridMap &subMap, const std::string &layer);


void setMarkerColor(visualization_msgs::Marker &marker, const float &r, const float &g, const float &b);

Eigen::Matrix3f getRotationMatrix(const float &roll, const float &pitch, const float &yaw);

Eigen::Matrix2d getRotationMatrix(const float &yaw);

grid_map::Polygon getStepPolygon(const float &yaw,const grid_map::Position &pos, const grid_map::Length &length);

void publishPolygon(const std::string &topicName, const grid_map::Polygon &polygon, ros::NodeHandle &nh, ros::Publisher &pubPolygon);

void fromPolygonMessage(const geometry_msgs::PolygonStamped &polStamped, grid_map::Polygon &polygon);

std::vector<eMatrixHom> interpolateNTransform(const eMatrixHom &h1, const eMatrixHom &h2, const int &N);

std::vector<eVector3> interpolateRPY(const eVector3 &rpy1, const eVector3 &rpy2, const int &N);

std::vector<eVector3> interpolateTranslation(const eVector3 &t1, const eVector3 &t2, const int &N);

void changeRotationDirection(eVector3 &rpy);

}

#endif // PLANNER_UTILS_H
