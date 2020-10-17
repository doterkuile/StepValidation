#include "planner_utils.h"



namespace plannerUtils
{

void setupMarker(visualization_msgs::Marker &marker, const std::string &headerFrame, const std::string &markerNameSpace)
{
  marker.header.frame_id = headerFrame;

  marker.header.stamp = ros::Time::now();
  marker.ns = markerNameSpace;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  plannerUtils::setMarkerColor(marker,0,1,0);
}

void setMarkerId(visualization_msgs::MarkerArray &markerArray)
{
  for(int ii{0}; ii < markerArray.markers.size(); ii++)
  {
    markerArray.markers[ii].id = ii;
  }
}


grid_map::Matrix flipPlane(const eVector3 &rpy,const eVector3 &stepPosition, const grid_map::GridMap &subMap, const std::string &layer)
{
    const grid_map::Matrix& plane = subMap.get("elevation");
    eMatrixRot R = matrixRollPitchYaw(rpy.x(), rpy.y(), 0.0);
    grid_map::Matrix flippedPlane(plane.rows(), plane.cols());
    for(grid_map::GridMapIterator iterator(subMap); !iterator.isPastEnd(); ++iterator)
    {
        grid_map::Index index = *iterator;
        eVector3 position;
        if(!subMap.getPosition3("elevation", index, position))
        {
            position.z() = 0.0;
            eVector2 position2D;
            subMap.getPosition(index,position2D);
            position.x() = position2D.x();
            position.y() = position2D.y();

        }
        position -=stepPosition;

        flippedPlane(index.x(), index.y()) = R.inverse().row(2) * position;
    }
    return flippedPlane;

}




std::vector<eVector3> getStepCorners(const eVector3 &stepPosition, const eVector3 &stepOrientation, const eVector3 &stepSize)
{
    // Rotation matrix
    eMatrixRot R = matrixRollPitchYaw(stepOrientation);

    std::vector<eVector3> cornerPositions;
    // upperleft
    cornerPositions.push_back(eVector3(stepSize.x()/2.0,
                                       stepSize.y()/2.0,
                                       0.0));
    //upperRight
    cornerPositions.push_back(eVector3(stepSize.x()/2.0,
                                      -stepSize.y()/2.0,
                                       0.0));

    // lowerRight
    cornerPositions.push_back(eVector3(-stepSize.x()/2.0,
                                       -stepSize.y()/2.0,
                                        0.0));
    // lowerLeft
    cornerPositions.push_back(eVector3(-stepSize.x()/2.0,
                                        stepSize.y()/2.0,
                                        0.0));

    std::vector<eVector3>::iterator corner = cornerPositions.begin();
    for(; corner !=cornerPositions.end(); corner++)
    {
        *corner = R * *corner + stepPosition;
    }

    return cornerPositions;

}



void setMarkerColor(visualization_msgs::Marker &marker, const float &r, const float &g, const float &b)
{
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0f;

}

bool transformCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud,  std::shared_ptr<tf2_ros::TransformListener> &tfListener, tf2_ros::Buffer &tfBuffer)
{
  geometry_msgs::TransformStamped transformStamped;

  tfListener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);
      try{

        transformStamped = tfBuffer.lookupTransform("odom", pointCloud->header.frame_id, ros::Time(0));
      }
      // Catch if base_link cannot be found
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ROS_WARN("Obstacles not published");
//        ros::Duration(1.0).sleep();
        return 0;
      }
  // Set rotation
  Eigen::Quaterniond q;
  q.x() = transformStamped.transform.rotation.x;
  q.y() = transformStamped.transform.rotation.y;
  q.z() = transformStamped.transform.rotation.z;
  q.w() = transformStamped.transform.rotation.w;
//  q = q.conjugate();
  // Set translation
  Eigen::Vector3d t;
  t.x() = transformStamped.transform.translation.x;
  t.y() = transformStamped.transform.translation.y;
  t.z() = transformStamped.transform.translation.z;

  pcl::transformPointCloud(*pointCloud, *pointCloud,t, q);
  pointCloud->header.frame_id = "/odom";

}

void getMaxIndex(const Eigen::MatrixXf &m, grid_map::Index &index)
{

  float maxValue = m.maxCoeffOfFinites();

  if(std::isnan(maxValue))
  {
      index.x() = ceil(m.rows()/2);
      index.y() = ceil(m.cols()/2);
    return;
  }

  for(int ii{0}; ii < m.rows(); ii++)
  {
    for(int jj{0}; jj < m.cols(); jj++)
    if(m(ii,jj) == maxValue)
    {
      index.x() = ii;
      index.y() = jj;
      return;
    }
  }
}

Eigen::Matrix3f getRotationMatrix(const float &roll, const float &pitch, const float &yaw)
{
  Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

  Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
  return q.toRotationMatrix();
}

Eigen::Matrix2d getRotationMatrix(const float &yaw)
{
  Eigen::Matrix2d m;
  m << std::cos(yaw), -std::sin(yaw),  std::sin(yaw), std::cos(yaw);
  return m;

}

grid_map::Polygon getStepPolygon(const float &yaw,const grid_map::Position &footPosition, const grid_map::Length &footSize)
{
  Eigen::Matrix2d R = plannerUtils::getRotationMatrix(yaw);

  // Footstep corners
  std::vector<grid_map::Position> vertices;

  auto start = std::chrono::high_resolution_clock::now();

  // frontLeft
  vertices.push_back(eVector2(footSize.x(), footSize.y())/2);
  // frontRight
  vertices.push_back(eVector2(footSize.x(), -footSize.y())/2);
  // backRight
  vertices.push_back(eVector2(-footSize.x(), -footSize.y())/2);
  // backLeft
  vertices.push_back(eVector2(-footSize.x(), footSize.y())/2);

  // Coordinate of each vertex in map frame
  std::vector<eVector2>::iterator vertex = vertices.begin();
  for(; vertex !=vertices.end(); vertex++)
  {
    *vertex = R * *vertex + footPosition;

  }

 grid_map::Polygon polygon(vertices);
 return polygon;

}

void publishPolygon(const std::string &topicName, const grid_map::Polygon &polygon, ros::NodeHandle &nh, ros::Publisher &pubPolygon)
{


  geometry_msgs::PolygonStamped polStamped;
  grid_map::PolygonRosConverter::toMessage(polygon, polStamped);
  pubPolygon.publish(polStamped);

}

void fromPolygonMessage(const geometry_msgs::PolygonStamped &polStamped, grid_map::Polygon &polygon)
{
  polygon.resetTimestamp();
  polygon.setFrameId(polStamped.header.frame_id);
  for( int ii{0}; ii < polStamped.polygon.points.size() ; ii++)
  {
    grid_map::Position vertex;
    vertex << polStamped.polygon.points[ii].x, polStamped.polygon.points[ii].y;

    polygon.addVertex(vertex);
  }

}

void setRightRotationDirection(eVector3 &rpy)
{
  for(int ii{0}; ii < rpy.size(); ii++)
  {
    if(rpy[ii] > M_PI/2)
    {
      rpy[ii] -= M_PI;
    }
    else if(rpy[ii] <- M_PI/2)
    {
      rpy[ii] += M_PI;
    }
  }


}

std::vector<eMatrixHom> interpolateNTransform(const eMatrixHom &h1, const eMatrixHom &h2, const int &N)
{
  std::vector<eMatrixHom> hVector;

  eVector3 p1 = h1.translation();
  eVector3 p2 = h2.translation();

  eQuaternion r1 = eQuaternion(h1.rotation());
  eQuaternion r2 = eQuaternion(h2.rotation());

  for(int ii{1}; ii <= N; ii++)
  {

    double slerpCoeff = static_cast<double>(ii)/N;
    eQuaternion rm = r1.slerp(static_cast<double>(ii)/N, r2);
    eVector3 pm = (p1 + p2)* ii / N;

    hVector.push_back(createMatrix(rm,pm));

  }

  return hVector;

}

std::vector<eVector3> interpolateTranslation(const eVector3 &t1, const eVector3 &t2, const int &N)
{
  std::vector<eVector3> tVector;


  for(int ii{1}; ii <= N; ii++)
  {

    eVector3 tm = t1 + (t2 - t1)* ii / N;

    tVector.push_back(tm);

  }

  return tVector;

}

std::vector<eVector3> interpolateRPY(const eVector3 &rpy1, const eVector3 &rpy2, const int &N)
{
  std::vector<eVector3> rVector;
  eVector3 rpym;

  Eigen::Quaterniond r1 = quaternionRollPitchYaw(rpy1);
  Eigen::Quaterniond r2 = quaternionRollPitchYaw(rpy2);


  for(int ii{1}; ii <= N; ii++)
  {
    double slerpCoeff = static_cast<double>(ii)/N;
    eQuaternion rm = r1.slerp(static_cast<double>(ii)/N, r2);
    extractRollPitchYaw(rm, &rpym.x(),&rpym.y(), &rpym.z());


    rVector.push_back(rpym);
  }

  return rVector;

}

void changeRotationDirection(eVector3 &rpy)
{
  if(rpy.x() > M_PI)
  {
    rpy.x() = rpy.x() -M_PI;
  }

}


void switchSide(pal_locomotion::Side &side)
{
  if(side._value == pal_locomotion::Side::LEFT)
  {
    side = pal_locomotion::Side::RIGHT;
  }
  else
  {
    side = pal_locomotion::Side::LEFT;
  }
}


void getNumberOfDifferentValues(const Eigen::MatrixXf &m, int &n)
{
  Eigen::MatrixXf matrix = m;
  float sum = matrix.sumOfFinites();
  n = 1;
  if(std::isnan(matrix.sumOfFinites()))
  {
    return;
  }
  float v1;
  while(!std::isnan(matrix.sumOfFinites()))
  {
     v1 = matrix.maxCoeffOfFinites();
    for(int ii{0}; ii < matrix.size(); ii++)
    {
      if((matrix(ii) == v1))
      {
        matrix(ii) = std::nan("1");
      }
    }
    n++;
    v1 = matrix.maxCoeffOfFinites();

  }

}


} // End of namespace
