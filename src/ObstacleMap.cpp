#include <pal_footstep_planner/foot_step_planning/ObstacleMap.h>
#include <ros/console.h>
#include <pal_ros_utils/conversions.h>
#include <pal_footstep_planner/utils.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

namespace pal{
namespace Footstep{

ObstacleMap::ObstacleMap(ros::NodeHandle &nh, const Eigen::VectorXd &cellsize,
                         const Eigen::VectorXi &num_angle_bins,
                         const std::pair<Eigen::VectorXd, Eigen::VectorXd> &min_max_angle_range,
                         const eVector3 &footsize)
  : nh_(nh),
    cellsize_(cellsize),
    num_angle_bins_(num_angle_bins),
    min_max_angle_range_(min_max_angle_range),
    footsize_(footsize)
{}

Map2D::Map2D(ros::NodeHandle &nh,
             const nav_msgs::OccupancyGridConstPtr &gridmap,
             const Eigen::VectorXd &cellsize,
             const Eigen::VectorXi &num_angle_bins,
             const std::pair<Eigen::VectorXd, Eigen::VectorXd> &min_max_angle_range,
             const eVector3 &footsize)
  : ObstacleMap(nh, cellsize, num_angle_bins, min_max_angle_range, footsize),
    map_(new gridmap_2d::GridMap2D(gridmap, false))
{
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 100);
}



bool Map2D::occupied(const FootStepPlanningState &s, bool check_collision) const
{
  if(!check_collision)
    return false;

  FootStepState state = s.getState(cellsize_, num_angle_bins_, min_max_angle_range_);

  Eigen::VectorXd pos = state.getPosition();

  eVector3 transl(pos[0], pos[1], 0);

  Eigen::VectorXd orientation = state.getOrientation();

  if(orientation.rows() != 1)
    throw std::runtime_error("Map2D::Occupied not implemented for this case");

  eMatrixRot rot_matrix = matrixRollPitchYaw(0, 0, orientation[0]);

  return footCollision(transl, rot_matrix);
}

void Map2D::visualizeMap() const
{
  nav_msgs::OccupancyGrid msg = map_->toOccupancyGridMsg();

  ros::Time start_chrono = ros::Time::now();

  while((ros::Time::now()-start_chrono) < ros::Duration(1.0))
  {
    map_pub_.publish(msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
}

// ATT! This function only checks the 4 points of the corner. It should be improved checking more points or calculating the incircle/outcircle
bool Map2D::footCollision(const eVector3 &foot_center, const eMatrixRot &rotation_matrix) const
{
  eVector3 corner1, corner2, corner3, corner4;
  eVector3 footsize_xy;

  if(map_->isOccupiedAt(foot_center[0], foot_center[1]))
    return true;

  footsize_xy << footsize_[0], footsize_[1], 0;
  corner1 = (rotation_matrix * footsize_xy) + foot_center;
  if(map_->isOccupiedAt(corner1[0], corner1[1]))
    return true;

  footsize_xy << -footsize_[0], footsize_[1], 0;
  corner2 = (rotation_matrix * footsize_xy) + foot_center;
  if(map_->isOccupiedAt(corner2[0], corner2[1]))
    return true;

  footsize_xy << footsize_[0], -footsize_[1], 0;
  corner3 = (rotation_matrix * footsize_xy) + foot_center;
  if(map_->isOccupiedAt(corner3[0], corner3[1]))
    return true;

  footsize_xy << -footsize_[0], -footsize_[1], 0;
  corner4 = (rotation_matrix * footsize_xy) + foot_center;
  if(map_->isOccupiedAt(corner4[0], corner4[1]))
    return true;

  return false;
}

Map3D::Map3D(ros::NodeHandle &nh, const OcTreePtr &octomap, const Eigen::VectorXd &cellsize,
             const Eigen::VectorXi &num_angle_bins,
             const std::pair<Eigen::VectorXd, Eigen::VectorXd> &min_max_angle_range,
             const eVector3 &footsize)
  : ObstacleMap(nh, cellsize, num_angle_bins, min_max_angle_range, footsize),
    map_(octomap)
{
  if(map_->getResolution() > cellsize_.minCoeff())
    throw std::runtime_error("The resolution from the octomap is higher than the cellsize");

  map_pub_ = nh_.advertise<octomap_msgs::Octomap>("/map", 100);
}

bool Map3D::occupied(const FootStepPlanningState &s, bool check_collision) const
{
  FootStepState state = s.getState(cellsize_, num_angle_bins_, min_max_angle_range_);

  Eigen::VectorXd foot_center = state.getPosition();

  if(foot_center.rows() != 3)
    throw std::runtime_error("Map3D non valid state");

  if(foot_center[2] < 0)
  {
    return true;
  }

  octomap::OcTreeNode* n = map_->search(foot_center[0], foot_center[1], foot_center[2]);

  double octomap_resolution = map_->getResolution();

  if(n == nullptr || !map_->isNodeOccupied(n))
  {
    octomap::OcTreeNode* n_lower = map_->search(foot_center[0], foot_center[1], foot_center[2]-octomap_resolution);

    if(n_lower == nullptr || !map_->isNodeOccupied(n_lower))
      return true;
    else
    {
      if(!check_collision)
        return false;
      Eigen::VectorXd orientation = state.getOrientation();
      if(orientation.rows() != 1)
        throw std::runtime_error("Map3D::Occupied not implemented for this case");
      eMatrixRot rotation_matrix = matrixRollPitchYaw(0, 0, orientation[0]);
      if(footCollision(foot_center, rotation_matrix))
        return true;
      else
        return false;
    }
  }
  else if(map_->isNodeOccupied(n) && check_collision)
    return true;
  else if(map_->isNodeOccupied(n) && (!check_collision))
  {
    octomap::OcTreeNode* n_upper = map_->search(foot_center[0], foot_center[1], foot_center[2]+octomap_resolution);
    if(n_upper == nullptr || !map_->isNodeOccupied(n_upper))
      return false;
    else
      return true;
  }
  else
    throw std::runtime_error("Invalid option");

  return true;
}

// OJO Faltaria tambe chequejar que el peu no quedi mig penjant a l'esglao
bool Map3D::footCollision(const eVector3 &foot_center, const eMatrixRot &rotation_matrix) const
{
  eVector3 corner1, corner2, corner3, corner4;
  eVector3 footsize_xy;
  octomap::point3d corner1_octomap, corner2_octomap, corner3_octomap, corner4_octomap;

  //  std::cerr << "Foot_center " << foot_center.transpose() << std::endl;
  footsize_xy << footsize_[0], footsize_[1], 0;
  corner1 = (rotation_matrix * footsize_xy) + foot_center;
  pal::convert(corner1, corner1_octomap);

  footsize_xy << -footsize_[0], footsize_[1], 0;
  corner2 = (rotation_matrix * footsize_xy) + foot_center;
  pal::convert(corner2, corner2_octomap);

  footsize_xy << footsize_[0], -footsize_[1], 0;
  corner3 = (rotation_matrix * footsize_xy) + foot_center;
  pal::convert(corner3, corner3_octomap);

  footsize_xy << -footsize_[0], -footsize_[1], 0;
  corner4 = (rotation_matrix * footsize_xy) + foot_center;
  pal::convert(corner4, corner4_octomap);

  //  std::cerr << "Corner1 " << corner1.transpose() << std::endl;
  //  std::cerr << "Corner2 " << corner2.transpose() << std::endl;
  //  std::cerr << "Corner3 " << corner3.transpose() << std::endl;
  //  std::cerr << "Corner4 " << corner4.transpose() << std::endl;

  // ATT! This checks that the foot is not half insise half outside the step
//    double map_resolution = map_->getResolution();

//    octomap::OcTreeNode* n = map_->search(corner1[0], corner1[1], corner1[2] - map_resolution);
//    if(n == nullptr || !map_->isNodeOccupied(n))
//      return true;

//    octomap::OcTreeNode* n1 = map_->search(corner2[0], corner2[1], corner2[2] - map_resolution);
//    if(n1 == nullptr || !map_->isNodeOccupied(n1))
//      return true;

//    octomap::OcTreeNode* n2 = map_->search(corner3[0], corner3[1], corner3[2] - map_resolution);
//    if(n2 == nullptr || !map_->isNodeOccupied(n2))
//      return true;

//    octomap::OcTreeNode* n3 = map_->search(corner4[0], corner4[1], corner4[2] - map_resolution);
//    if(n3 == nullptr || !map_->isNodeOccupied(n3))
//      return true;

  // OJO!! Revisar aixo que segurament trigara molt!!!!
  octomap::KeyRay keys;
  map_->computeRayKeys(corner1_octomap, corner2_octomap, keys);
  if(KeyValues(keys))
    return true;
  else
    keys.reset();

  map_->computeRayKeys(corner1_octomap, corner3_octomap, keys);
  if(KeyValues(keys))
    return true;
  else
    keys.reset();

  map_->computeRayKeys(corner1_octomap, corner4_octomap, keys);
  if(KeyValues(keys))
    return true;
  else
    keys.reset();

  map_->computeRayKeys(corner3_octomap, corner2_octomap, keys);
  if(KeyValues(keys))
    return true;
  else
    keys.reset();

  map_->computeRayKeys(corner2_octomap, corner4_octomap, keys);
  if(KeyValues(keys))
    return true;
  else
    keys.reset();
  map_->computeRayKeys(corner4_octomap, corner3_octomap, keys);
  if(KeyValues(keys))
    return true;

  return false;
}

bool Map3D::KeyValues(const octomap::KeyRay &keys) const
{
  for(octomap::KeyRay::const_iterator it = keys.begin(); it != keys.end(); it++)
  {
    octomap::OcTreeNode* n = map_->search(*it);
    if(n == nullptr)
      continue;
    if(map_->isNodeOccupied(n))
      return true;
  }
  return false;
}

void Map3D::visualizeMap() const
{
  octomap_msgs::Octomap msg;
  octomap_msgs::binaryMapToMsg(*map_, msg);
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();

  ros::Time start_chrono = ros::Time::now();

  while((ros::Time::now()-start_chrono) < ros::Duration(1.0))
  {
    map_pub_.publish(msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
}

} // namespace Footstep
} // namespace pal
