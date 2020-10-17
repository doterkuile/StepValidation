#include "Footstep6D.h"


Footstep6D::Footstep6D()
{

    position_ = eVector3(0.0, 0.0, 0.0);
    rpy_ = eVector3(0.0, 0.0, 0.0);
    footSize_ = eVector3(0.2, 0.1, 0.05);
    setSide_ = false;
    side_ = pal_locomotion::Side::RIGHT;
    validStep_ = false;
}



Footstep6D::Footstep6D(const Eigen::Vector3d &pos, const Eigen::Quaterniond &q)
{
  position_ = pos;
  Eigen::Vector3d rpyTemp = q.toRotationMatrix().eulerAngles(0,1,2);
  this->setOrientation(rpyTemp);

  footSize_ << 0.2,0.1,0.05 ;
  setSide_ = false;

}


Footstep6D::Footstep6D(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation, const Eigen::Vector3d &size)
{
  position_ = position;
  this->setOrientation(orientation);
  footSize_ = size;
  setSide_ = false;
}

Footstep6D::Footstep6D(const Eigen::VectorXd &position, const Eigen::VectorXd &orientation, const eVector3 &size, const pal_locomotion::Side &side)
     :footSize_(size),
      setSide_(false)
{
     this->setPosition(position);
     this->setOrientation(orientation);
     this->setSide(side);

}


Footstep6D::Footstep6D(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation, const Eigen::Vector3d &size, const pal_locomotion::Side &side):
    setSide_(false)
{
  position_ = position;
  this->setOrientation(orientation);
  footSize_ = size;


  this->setSide(side);
}

Footstep6D::Footstep6D(const Footstep6D &step)
{
    rpy_ = step.rpy_;
    position_  = step.position_;
    footSize_ = step.footSize_;
    validStep_ = step.validStep_;
    side_ = step.side_;
    setSide_ = step.setSide_;
}

Footstep6D::Footstep6D(const Eigen::Isometry3d &pose, const Eigen::Vector3d &size):
    setSide_(false)
{
  footSize_ = size;
  eVector3 rpyTemp;
  extractRollPitchYaw(pose, &rpyTemp.x(), &rpyTemp.y(), &rpyTemp.z());
  this->setOrientation(rpyTemp);
//  ROS_INFO_STREAM("Inside Footstep6D constructor");
//  std::cout << "Rpy of Footstep6D is " << rpy_ << '\n';

}

Footstep6D::Footstep6D(const Eigen::Isometry3d &pose, const Eigen::Vector3d &size, const pal_locomotion::Side &side):
    setSide_(false)
{
  footSize_ = size;
  position_ = pose.translation();
  eVector3 rpyTemp;
  extractRollPitchYaw(pose, &rpyTemp.x(), &rpyTemp.y(), &rpyTemp.z());
  this->setOrientation(rpyTemp);
//  ROS_INFO_STREAM("Inside Footstep6D constructor");
//  std::cout << "Rpy of Footstep6D is " << rpy_ << '\n';
  this->setSide(side);

}



Footstep6D::Footstep6D(const Eigen::Vector3d &position, const Eigen::Quaterniond &q, const Eigen::Vector3d &size)
{
  position_ = position;
  eVector3 rpyTemp;
  extractRollPitchYaw(q, &rpyTemp.x(), &rpyTemp.y(), &rpyTemp.z());
  this->setOrientation(rpyTemp);
  footSize_ = size;
  setSide_ = false;

}


void Footstep6D::validateStep(const bool &validStep)
{
  validStep_ = validStep;
}


Eigen::Vector2d Footstep6D::get2DPosition() const
{
  Eigen::Vector2d pos(this->getX(),this->getY());
  return pos;

}

void Footstep6D::changeSide(const pal_locomotion::Side &side)
{
  side_ = side;
}
void Footstep6D::setSide(const pal_locomotion::Side &side)
{

  if(setSide_)
  {
      if(side._value == pal_locomotion::Side::LEFT)
      {
          std::cout << "stepside is LEFT" << '\n';
      }
      else
      {
          std::cout << "stepside is RIGHT" << '\n';

      }

    ROS_WARN_STREAM("Side has already been set, use function changeSide instead please");
    return;
  }
  side_ = side;
  setSide_ = true;

}

Eigen::Vector3d Footstep6D::getSize() const
{
  return footSize_;
}

void Footstep6D::setSize(const Eigen::Vector3d &size)
{
  footSize_ = size;
}

void Footstep6D::setHeight( float &h)
{

  if(std::isnan(h))
  {
   h = 0;
  }
  position_.z() = h;
}


void Footstep6D::stepToMarker(visualization_msgs::Marker &marker, const std::string &headerFrame, const std::string &markerNameSpace) const
{
  plannerUtils::setupMarker(marker, headerFrame, markerNameSpace);

  Eigen::Quaterniond q =  quaternionRollPitchYaw(rpy_);
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();


  eVector3 correctForMarkerCenter = eVector3(0.0,0.0,footSize_.z()/2);
  correctForMarkerCenter = q.toRotationMatrix() * correctForMarkerCenter;

  eVector3 position = correctForMarkerCenter + position_;

  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.pose.position.z = position.z();



  marker.scale.x = footSize_.x();
  marker.scale.y = footSize_.y();
  marker.scale.z = footSize_.z();


  if(!validStep_)
  {
    plannerUtils::setMarkerColor(marker,1,0,0);
  }

}

void Footstep6D::setOrientation(float &roll, float &pitch)
{
  if(std::isnan(roll) || std::isnan(pitch))
  {
    roll = 0;
    pitch = 0;
  }

 rpy_.x() = roll;
 rpy_.y() = pitch;
}

void Footstep6D::setOrientation(const Eigen::VectorXd &rpy)
{

     if(rpy.rows() == 1)
     {
          rpy_ = eVector3(0.0,0.0, rpy[0]);
     }
     else
     {
          Eigen::Vector3d rpyTemp(rpy);
           if(std::isnan(rpyTemp.x()) || std::isnan(rpyTemp.y()))
           {
             rpyTemp.x() = 0;
             rpyTemp.y() = 0;
           }
          rpy_ = rpyTemp;
     }

}

void Footstep6D::setPosition(const Eigen::VectorXd &position)
{
     if(position.rows() == 2)
     {
          position_ = eVector3(position.x(),position.y(),0.0);
     }
     else
     {
          position_ = position;
     }
}


Eigen::Vector3d Footstep6D::getRPY() const
{
  return rpy_;
}

Eigen::Vector3d Footstep6D::getPosition() const
{
  return position_;
}


double Footstep6D::getX() const
{
  return position_.x();
}

double Footstep6D::getY() const
{
  return position_.y();
}

double Footstep6D::getHeight() const
{
  return position_.z();
}

double Footstep6D::getYaw() const
{

  return rpy_.z();
}

double Footstep6D::getRoll() const
{
  return rpy_.x();
}

double Footstep6D::getPitch() const
{

  return rpy_.y();
}

eMatrixHom Footstep6D::getPose() const
{
  return createMatrix(rpy_, position_);
}

pal_locomotion::Side Footstep6D::getSide() const
{
  return side_;
}

bool Footstep6D::isValid() const
{
    return validStep_;
}
bool Footstep6D::hasSide() const
{
    if(setSide_)
    {
        ROS_INFO_STREAM("step side has already been set");
    }
    else {
        ROS_WARN_STREAM("No side available");
    }
    return setSide_;
}
