#include "StepValidation.h"


namespace StepValidator{


void StanceLeg::setStep(const Footstep6D &step)
{
    step_ = step;
}

std::vector<grid_map::Position> StanceLeg::getVertices()
{
    if(vertices_.size() == 0)
    {
        grid_map::Length footSize = 1.5 * eVector2(step_.getSize().x(),
                                             step_.getSize().y());
        grid_map::Polygon polygon = plannerUtils::getStepPolygon(step_.getYaw(),step_.get2DPosition(), footSize);
        vertices_ = polygon.getVertices();
    }

    return vertices_;
}

double StanceLeg::getHeight() const
{
    return step_.getHeight();
}

grid_map::Position StanceLeg::get2DPosition() const
{
    return step_.get2DPosition();
}




Footstep6D StanceLeg::getStep() const
{
    return step_;
}


NextStep::NextStep()
{
    hasStepPolygon_ = false;
    hasSidePolygon_ = false;
    hasFrontPolygon_ = false;
    hasEnlargedPolygon_ = false;
    hasMaxHeightIndex_ = false;
    hasSubmap_ = false;
}

void NextStep::setStep(const Footstep6D &step)
{
    step_ = step;
    validStep_ = true;
    validStanceLeg_= true;
    hasStepPolygon_ = false;
    hasSidePolygon_ = false;
    hasFrontPolygon_ = false;
    hasEnlargedPolygon_ = false;
    hasMaxHeightIndex_ = false;
    hasSubmap_ = false;
}

Footstep6D NextStep::getStep()
{

    if(validStep_ && validStanceLeg_)
    {
        step_.validateStep(true);
    }
    else
    {
        step_.validateStep(false);
    }

    return step_;
}

void NextStep::validateStep(const bool &isValid)
{
    validStep_ = isValid;
}

void NextStep::hasValidStanceLeg(const bool &isValid)
{
    validStanceLeg_ = isValid;
}

void NextStep::setStepPolygon(const grid_map::Polygon &polygon)
{
    stepPolygon_ = polygon;
    hasStepPolygon_ = true;
}
grid_map::Polygon NextStep::getStepPolygon()
{
    if(!hasStepPolygon_)
    {
        grid_map::Length footSize = eVector2(step_.getSize().x(),
                                             step_.getSize().y());
        stepPolygon_ = plannerUtils::getStepPolygon(step_.getYaw(),step_.get2DPosition(), footSize);
        hasStepPolygon_ = true;
    }

    return stepPolygon_;

}


grid_map::Polygon NextStep::getSidePolygon()
{
    if(!hasSidePolygon_)
    {

        Eigen::Matrix2d rotationMatrix = plannerUtils::getRotationMatrix(step_.getYaw());
        grid_map::Position sidePolygonPosition = eVector2(0.0, 7.0 / 8.0 * step_.getSize().y());
        if(step_.getSide()._value == pal_locomotion::Side::RIGHT)
        {
            sidePolygonPosition.y() = - sidePolygonPosition.y();

        }
        sidePolygonPosition = rotationMatrix * sidePolygonPosition + step_.get2DPosition();


        grid_map::Length sidePolygonSize = 3.0 / 4.0 * eVector2(step_.getSize().x(),
                                                                step_.getSize().y());
        sidePolygon_ = plannerUtils::getStepPolygon(step_.getYaw(),sidePolygonPosition, sidePolygonSize);
        hasSidePolygon_ = true;
    }

    return sidePolygon_;
}


grid_map::Polygon NextStep::getFrontPolygon()
{
    if(!hasFrontPolygon_)
    {

        Eigen::Matrix2d rotationMatrix = plannerUtils::getRotationMatrix(step_.getYaw());
        grid_map::Position frontPolygonPosition = eVector2(step_.getSize().x()/2 + step_.getSize().y()/2, step_.getSize().y()/2);

        if(step_.getSide()._value == pal_locomotion::Side::RIGHT)
        {
            frontPolygonPosition.y() = - frontPolygonPosition.y();

        }
        frontPolygonPosition = rotationMatrix * frontPolygonPosition + step_.get2DPosition();

        grid_map::Length frontPolygonSize = eVector2(step_.getSize().y()/2.0,
                                                     2.0 * step_.getSize().y());
        grid_map::Polygon frontPolygon = plannerUtils::getStepPolygon(step_.getYaw(),frontPolygonPosition, frontPolygonSize);
        hasFrontPolygon_ = true;
    }

    return frontPolygon_;
}

void NextStep::setSubmap(const grid_map::GridMap &submap)
{
    submap_ = submap;
    hasSubmap_ = true;
}

grid_map::GridMap NextStep::getSubmap()
{
    return submap_;
}



grid_map::Polygon NextStep::getEnlargedStepPolygon(const Eigen::VectorXd cellSize)
{
    if(!hasEnlargedPolygon_)
    {



        eVector3 size = step_.getSize();

        // Increase checking area with a cell at all sides
        size.y() += 2.0 * cellSize.y();
        size.x() += 2.0 * cellSize.x();

        eVector2 polygonStepSize(step_.getSize().x(), step_.getSize().y());
        enlargedPolygon_ = plannerUtils::getStepPolygon(step_.getYaw(), step_.get2DPosition(), polygonStepSize);
        hasEnlargedPolygon_ = true;
    }

    return enlargedPolygon_;
}

double NextStep::getHeight() const
{
    return step_.getHeight();
}

eVector2 NextStep::get2DPosition() const
{
    return step_.get2DPosition();
}

eVector3 NextStep::getPosition() const
{
    return step_.getPosition();
}

eVector3 NextStep::getOrientation() const
{
    return step_.getRPY();
}

void NextStep::setMaxHeightIndex(const grid_map::Index &maxIndex)
{
    maxHeightIndex_ = maxIndex;
    hasMaxHeightIndex_ = true;
}

grid_map::Index NextStep::getMaxHeightIndex()
{
    if(!hasSubmap_)
    {
        ROS_ERROR_STREAM("No submap was set");
    }
    if(!hasStepPolygon_)
    {
        ROS_ERROR_STREAM("No stepPolygon was set");
    }

    if(!hasMaxHeightIndex_)
    {
        grid_map::GridMap::Matrix heightMap = submap_.get("elevation");

        grid_map::Index index;

        submap_.getIndex(step_.get2DPosition(),index);

        plannerUtils::getMaxIndex(heightMap,index);

        grid_map::Position position;
        submap_.getPosition(index,position);


        while(!stepPolygon_.isInside(position))
        {
            heightMap(index.x(),index.y()) = std::nan("1");
            plannerUtils::getMaxIndex(heightMap,index);
            submap_.getPosition(index,position);
        }
        maxHeightIndex_ = index;
        hasMaxHeightIndex_ = true;
    }
    return maxHeightIndex_;
}




} // End of namespace StepValidator





StepValidation::StepValidation(ros::NodeHandle &nh):
    nh_(nh),
    resetStanceLeg_(true),
    resetStep_(true)
{

    this->setupParams();
    this->setSubscriber("/elevation_mapping/elevation_map_raw");



}

StepValidation::StepValidation(ros::NodeHandle &nh, const boost::shared_ptr<grid_map::GridMap> &gridMap):
    nh_(nh),
    map_(gridMap),
    resetStanceLeg_(true),
    resetStep_(true)
{
    cellSize_.x() = map_->getResolution();
    cellSize_.y() = map_->getResolution();
    this->setupParams();

}




StepValidation::StepValidation(ros::NodeHandle &nh, const FootstepArray &stepArray):
    nh_(nh),
    footstepArray_(stepArray),
    resetStanceLeg_(true),
    resetStep_(true)
{
    this->setupParams();
    this->setSubscriber("/elevation_mapping/elevation_map_raw");

}


void StepValidation::setupPublishers()
{
    pubMarker_ = nh_.advertise<visualization_msgs::MarkerArray>("/footstepmarker", 1);
    pubGridMap_ = nh_.advertise<grid_map_msgs::GridMap>("/ElevationMap/SubMap", 10);
    pubStep_ = nh_.advertise<visualization_msgs::Marker>("/SingleStep", 20);
    pubPolygon_ = nh_.advertise<geometry_msgs::PolygonStamped>("/stepPolygons/polygon", 20);
    pubWiggledPolygon_ = nh_.advertise<geometry_msgs::PolygonStamped>("/testings", 20);
    if(nodeExpansion_)
    {
        pubStepExpansionList_ = nh_.advertise<visualization_msgs::Marker>("/StepValidation/expansionList",20);
        pubStepExpansionArray_ = nh_.advertise<visualization_msgs::MarkerArray>("/StepValidation/expansionArray",20);
    }

}

void StepValidation::setupParams()
{
    this->setupPublishers();
    if(!nh_.getParam("/step_validation_config", config_))
    {
        ROS_ERROR_STREAM("Could not retrieve footstep planner configuration, results might not be valid");
    }

    footSize_ =                 eVector3(config_["foot"]["size"]["x"],
            config_["foot"]["size"]["y"],
            config_["foot"]["size"]["z"]);

    maxStepTranslation_  =      config_["step_translation"]["max"]["y"];

    maxHeight_ = config_["validation"]["max_height_difference"];
    stepCollisionHeight_ = config_["validation"]["step_collision_height"];
    shinCollisionHeight_ = config_["validation"]["shin_collision_height"];

    maxIncline_ = config_["validation"]["max_incline"];
    minFoothold_ = config_["validation"]["partial_foothold"]["min_foothold"];
    partialFootholdHeightDeviation_ =config_["validation"]["partial_foothold"]["height_deviation"];

    nh_.param("/full_map_expansion", expandFullMap_, false);

    nh_.param("/node_expansion", nodeExpansion_, false);


    closestStepSearchArea_ = eVector2(static_cast<double>(config_["expansion"]["length"]["x"]),
            static_cast<double>(config_["expansion"]["length"]["y"]));

}

void StepValidation::updateMap(const boost::shared_ptr<grid_map::GridMap> &gridMap)
{
    boost::lock_guard<boost::recursive_mutex> guard(mutexMap_);

    map_ = gridMap;
    cellSize_.x() = map_->getResolution();
    cellSize_.y() = map_->getResolution();




}

void StepValidation::mapCallBack(const grid_map_msgs::GridMap &inputMsg)
{
    boost::lock_guard<boost::recursive_mutex> guard(mutexMap_);
    map_ = boost::make_shared<grid_map::GridMap>();
    ROS_INFO_STREAM("Received elevation map, inside callback");
    receivedMap_ = true;
    cellSize_.x() = inputMsg.info.resolution;
    cellSize_.y() = inputMsg.info.resolution;

    grid_map::GridMapRosConverter::fromMessage(inputMsg, *map_);


}

void StepValidation::polygonCallBack(const custom_planner_msgs::polygonArray &polygonArray)
{
    boost::lock_guard<boost::recursive_mutex> guard(mutexMap_);

    polygonArray_ = polygonArray;
    std::cout << "inside polygoncallbakc with size = " << polygonArray.polygonArray.size() << '\n';

}

boost::shared_ptr<grid_map::GridMap> StepValidation::getGridMap() const
{
    return map_;
}

bool StepValidation::isSubscribed()
{
    return subMap_.getNumPublishers() > 0;
}

void StepValidation::setSubscriber(const std::string &s)
{
    subMap_ = nh_.subscribe(s, 10, &StepValidation::mapCallBack, this);
    subPolygonArray_ = nh_.subscribe("/elevation_mapping/Obstacle/polygons", 1000, &StepValidation::polygonCallBack, this);

}

void StepValidation::setStepArray(FootstepArray &stepArray)
{
    footstepArray_ = stepArray;
}

void StepValidation::setStanceLeg(Footstep6D &stanceLeg)
{
    this->snapStepToEnvironment(stanceLeg);

    stanceLeg_.setStep(stanceLeg);


}

void StepValidation::resetStanceLeg(const bool resetLeg)
{
    resetStanceLeg_ = resetLeg;
}

void StepValidation::setNewStep(Footstep6D &step)
{
    step.validateStep(true);
    grid_map::Length footSize = eVector2(step.getSize().x(),
                                         step.getSize().y());

    grid_map::Polygon polygon = plannerUtils::getStepPolygon(step.getYaw(),step.get2DPosition(), footSize);
    bool isSucces;
    grid_map::GridMap submap = this->orientedSubMap(polygon, isSucces);

    grid_map::Index maxIndex;

    submap.getIndex(step.get2DPosition(),maxIndex);

    this->getMaxHeightIndex(submap, polygon, maxIndex);

    this->nodeSnapping(step, maxIndex,submap);
    nextStep_.setStep(step);
    nextStep_.setStepPolygon(polygon);
    nextStep_.setSubmap(submap);
    nextStep_.setMaxHeightIndex(maxIndex);


}

void StepValidation::resetNextStep(const bool resetStep)
{
    resetStep_ = resetStep;
}


FootstepArray StepValidation::getValidatedSteps(const bool &findClosestValidStep)
{
    boost::lock_guard<boost::recursive_mutex> guard(mutexMap_);

    if(footstepArray_.size() == 0)
    {
        ROS_ERROR_STREAM("No steps received, cannot validate");
        FootstepArray emptyArray;
        return emptyArray;

    }
    else
    {

        if(!map_)
        {
            ROS_WARN_STREAM("No map received, footsteps are not validated");
            validatedStepArray_ = footstepArray_;

        }
        else
        {
            ROS_WARN_STREAM("Map received, footsteps are validated");
            validatedStepArray_ = footstepArray_;
            int stepCount = 0;

            for (FootstepArray::iterator step = validatedStepArray_.begin(); step != validatedStepArray_.end(); ++step)
            {
                boost::shared_ptr<Footstep6D> previousStep = this->getPreviousStep(validatedStepArray_, step);

                this->snapAndValidate(*previousStep, *step, findClosestValidStep);
                stepCount++;
                if(!expandFullMap_){
                    std::cout << "step nr " << stepCount <<'\n';
                }
            } // end step iteration//
            this->publishMarker(validatedStepArray_);

        }

    }



    return validatedStepArray_;

}


bool StepValidation::getValidatedSteps(Footstep6D &stanceFoot, Footstep6D &nextFoot)
{
    boost::lock_guard<boost::recursive_mutex> guard(mutexMap_);

    bool findClosestValidStep = false;

    if(!map_)
    {
        // No map set cannot validate or snap step
        return false;
    }
    else
    {
        this->snapAndValidate(stanceFoot, nextFoot, findClosestValidStep);
    }
    return true;


}


void StepValidation::snapAndValidate(Footstep6D &stanceFoot, Footstep6D &nextStep, const bool &findClosestValidStep)
{

    if(resetStanceLeg_)

    {
        this->setStanceLeg(stanceFoot);
        nextStep_.hasValidStanceLeg(true);
    }

    if(resetStep_)
    {
        this->setNewStep(nextStep);
    }

    this->validateStep(stanceLeg_, nextStep_);

    stanceFoot = stanceLeg_.getStep();
    nextStep = nextStep_.getStep();

    if(findClosestValidStep && !nextStep_.getStep().isValid())
    {
        nextStep_.getStep() = this->findClosestStep(nextStep_.getStep());

    }


}




FootstepArray StepValidation::getValidatedSteps(FootstepArray &stepArray, const bool &findClosestValidStep)
{
    boost::lock_guard<boost::recursive_mutex> guard(mutexMap_);
    FootstepArray validatedStepArray;

    if(stepArray.size() == 0)
    {
        ROS_ERROR_STREAM("No steps received, cannot validate");
        FootstepArray emptyArray;
        return emptyArray;

    }
    else
    {
        if(!map_)
        {

            //          ROS_WARN_STREAM("No map received, footsteps are not validated");
            validatedStepArray = stepArray;
        }
        else
        {
            validatedStepArray = stepArray;

            for (FootstepArray::iterator step = validatedStepArray.begin(); step != validatedStepArray.end(); ++step)
            {
                boost::shared_ptr<Footstep6D> previousStep = this->getPreviousStep(validatedStepArray, step);

                this->snapAndValidate(*previousStep, *step, findClosestValidStep);

            } // end step iteration

        } // end check if map is present

    } // check if step array is valid


    return validatedStepArray;

}

void StepValidation::validateStep(const StepValidator::StanceLeg &stanceLeg, StepValidator::NextStep &nextStep)
{

//    this->pubStep(currentStep);

    if(!nextStep.getStep().isValid())
    {
        return;
    }


    if(!this->checkHeight(nextStep.getStep()))
    {


        if(!expandFullMap_){
            ROS_ERROR_STREAM("The stepheight exceeded the maximum");
        }
        nextStep.hasValidStanceLeg(false);

        return;
    }

    if(!nodeExpansion_)
    {
        if(!this->checkStanceLegClearance(nextStep.getStepPolygon()))
        {
            if(!expandFullMap_){
            ROS_ERROR_STREAM("The next step collides with the stanceleg");
            }
            nextStep.hasValidStanceLeg(false);
            return;
        }
    }


    if(!this->checkPartialFoothold(nextStep.getStepPolygon(), nextStep.getStep(), nextStep.getSubmap(), nextStep.getMaxHeightIndex()))
    {
        if(!expandFullMap_){
            ROS_ERROR_STREAM("The partial foothold was too low");
        }
        nextStep.validateStep(false);
        return;
    }


    grid_map::Polygon enlargedPolygon = nextStep.getEnlargedStepPolygon(cellSize_);
    if(!this->checkStepCollision(nextStep.getStep(), enlargedPolygon))
    {
        if(!expandFullMap_){
            ROS_ERROR_STREAM("The step collides with an obstacle");
        }
        nextStep.validateStep(false);

        return;
    }

    grid_map::Polygon sidePolygon = nextStep.getSidePolygon();
    grid_map::Polygon frontPolygon = nextStep.getFrontPolygon();



    if(!this->checkShinCollision(nextStep.getStep(),frontPolygon, sidePolygon))
    {
        if(!expandFullMap_){
            ROS_ERROR_STREAM("There is an object too close in front of the step");
        }
        nextStep.validateStep(false);

        return;
    }




//    if(polygonArray_.polygonArray.size() !=0)
//    {
//        //      this->increaseFoothold(currentStep,currentStepPolygon);


//    }



}


void StepValidation::snapStepToEnvironment(Footstep6D &step) const
{
    grid_map::Length footSize = eVector2(step.getSize().x(),
                                         step.getSize().y());

    grid_map::Polygon polygon = plannerUtils::getStepPolygon(step.getYaw(),step.get2DPosition(), footSize);
    polygon.setFrameId(map_->getFrameId());

    bool isSucces;
    grid_map::GridMap submap = this->orientedSubMap(polygon, isSucces);


    grid_map::Index maxIndex;
    submap.getIndex(step.get2DPosition(),maxIndex);

    if(!isSucces)
    {
        std::cout << "snap step to environment get submap is not succesfull" << '\n';
        step.validateStep(false);
        return;
    }






    this->getMaxHeightIndex(submap, polygon, maxIndex);

    //    int diffPlanes = this->getDifferentPlanes(submap);

    this->nodeSnapping(step, maxIndex,submap);
}

void StepValidation::nodeSnapping(Footstep6D &step,  const grid_map::Index &maxHeightIndex, const grid_map::GridMap &subMap) const
{


    float roll, pitch, height;

    height = subMap.at("elevation", maxHeightIndex);
    roll = subMap.at("roll", maxHeightIndex);
    pitch = subMap.at("pitch", maxHeightIndex);


    if(std::isnan(roll))
    {
        roll = 0.0;
        pitch = 0.0;
        height = 0.0;
    }


    Eigen::Matrix3d rotationMatrix = matrixRollPitchYaw(roll,pitch,0.0);

    grid_map::Position heightPos;
    subMap.getPosition(maxHeightIndex,heightPos);

    // Relative position on foot of maximum height
    Eigen::Vector3d heightPosition = Eigen::Vector3d(heightPos.x() - step.get2DPosition().x(),
                                                     heightPos.y()- step.get2DPosition().y(),
                                                     0.0 );
    heightPosition = rotationMatrix*heightPosition;
    if(roll < M_PI/2.0 || pitch < M_PI/2.0)
    {
        height -= heightPosition.z();
    }
    //}
    step.setHeight(height);
    Eigen::Vector3d rpy = eVector3(roll, pitch, step.getYaw());
    step.setOrientation(rpy);


    if(!this->checkIncline(step))
    {
        if(!expandFullMap_)
        {
            ROS_ERROR_STREAM("The stepincline exceeded the maximum");
        }
        step.validateStep(false);
    }


}

eVector3 StepValidation::getWayPoint(const Footstep6D &currentStep, const Footstep6D &nextStep) const
{
    eVector2 footSize =   eVector2(currentStep.getSize().x(),currentStep.getSize().y() * 1.3);
    grid_map::Polygon currentPolygon = plannerUtils::getStepPolygon(currentStep.getYaw(), currentStep.get2DPosition(),footSize);
    grid_map::Polygon nextPolygon = plannerUtils::getStepPolygon(nextStep.getYaw(), nextStep.get2DPosition(),footSize);

    std::vector<grid_map::Position> vertices;
    vertices.push_back(currentPolygon.getVertices()[0]);
    vertices.push_back(currentPolygon.getVertices()[1]);
    vertices.push_back(nextPolygon.getVertices()[2]);
    vertices.push_back(nextPolygon.getVertices()[3]);


    grid_map::Polygon polygon(vertices);

    bool isSucces;
    grid_map::GridMap subMap = this->orientedSubMap(polygon, isSucces);
    grid_map::GridMap::Matrix& heightMatrix = subMap.get("elevation");

    double maxGradient = 0;
    eVector3 wayPoint = (currentStep.getPosition() + nextStep.getPosition())/2.0;
    wayPoint.z() = std::max(currentStep.getHeight(), nextStep.getHeight());
    if(!isSucces || ((currentStep.getPosition() - nextStep.getPosition()).norm() < cellSize_.norm()))
    {
        return wayPoint;
    }



    for(grid_map::GridMapIterator iterator(subMap); !iterator.isPastEnd(); ++iterator)
    {
        grid_map::Index index = *iterator;
        eVector2 position;
        subMap.getPosition(index,position);

        if(!polygon.isInside(position))
        {
            continue;
        }

        eVector2 relativePosition = position - currentStep.get2DPosition();


        double gradient = heightMatrix(index.x(), index.y())/ relativePosition.norm();

        double wayPointHeight = std::max(heightMatrix.maxCoeffOfFinites(), static_cast<float>(currentStep.getHeight()));
        if(currentStep.getHeight() > nextStep.getHeight())
        {
            if(gradient > maxGradient)
            {
                maxGradient = gradient;
                wayPoint = eVector3(position.x(), position.y(),wayPointHeight);
            }

        }
        else
        {

            if(gradient < maxGradient)
            {
                maxGradient = gradient;
                wayPoint = eVector3(position.x(), position.y(),wayPointHeight);
            }
        }

    }


    // Project waypoint to trajactory of step
    eVector2 S2 = nextStep.get2DPosition();
    eVector2 S1 = currentStep.get2DPosition();
    eVector2 P = eVector2(wayPoint.x(), wayPoint.y());
    eVector2 a = P - S1;
    eVector2 b = S2 - S1;

    eVector2 P2 = S1 + static_cast<double>((a.transpose()*b)) / static_cast<double>((b.transpose()*b)) * b;

    wayPoint.x() = P2.x();
    wayPoint.y() = P2.y();

    return wayPoint;



}

void StepValidation::increaseFoothold(Footstep6D &step, const grid_map::Polygon &footPolygon)
{
    std::vector<geometry_msgs::PolygonStamped>::iterator iterator = polygonArray_.polygonArray.begin();
    for( ; iterator != polygonArray_.polygonArray.end(); iterator++)
    {
        grid_map::Polygon polygon;
        plannerUtils::fromPolygonMessage(*iterator, polygon);

        if(polygon.isInside(step.get2DPosition()))
        {
            FootWiggle wiggle(step, footPolygon, polygon, nh_, pubWiggledPolygon_);
            if(wiggle.foundWiggledStep())
            {
                footstepArray_.push_back(wiggle.getWiggledStep());
            }
            ROS_INFO_STREAM("Footstep is inside obstacle polygon");
            std::cout << "polygon size = " << polygon.getVertices().size() << '\n';
            return;
        }

    }

}

bool StepValidation::checkStepCollision(const Footstep6D &step, grid_map::Polygon &polygon) const
{
    bool isSucces;
    grid_map::GridMap subMap = this->orientedSubMap(polygon, isSucces);

    grid_map::Matrix flippedPlane = plannerUtils::flipPlane(step.getRPY(), step.getPosition(), subMap, "elevation");

    if(flippedPlane.maxCoeff() > stepCollisionHeight_)
    {
        return 0;
    }

    return 1;
}


bool StepValidation::checkShinCollision(const Footstep6D &step, grid_map::Polygon &frontPolygon, grid_map::Polygon &sidePolygon) const
{

    bool isSucces;
    grid_map::GridMap submap = this->orientedSubMap(frontPolygon, isSucces);

    if(!isSucces)
    {
        return !isSucces;
    }

    // add side polygon to submap
    grid_map::GridMap sideMap = this->orientedSubMap(sidePolygon, isSucces);

    if(!isSucces)
    {
        return !isSucces;
    }

    grid_map::Matrix heightMapFlipped = plannerUtils::flipPlane(step.getRPY(), step.getPosition(), submap, "elevation");

    grid_map::Matrix heightSideMapFlipped = plannerUtils::flipPlane(step.getRPY(), step.getPosition(), sideMap, "elevation");


    if(heightMapFlipped.maxCoeff() > shinCollisionHeight_)
    {
        return 0;
    }

    if(heightSideMapFlipped.maxCoeff() > shinCollisionHeight_)
    {
        return 0;
    }



    return 1;


}


bool StepValidation::checkPartialFoothold(const grid_map::Polygon &polygon, const Footstep6D &step, const grid_map::GridMap &submap, const grid_map::Index &maxHeightIndex) const
{



    double factor = minFoothold_;
    double heightError =  partialFootholdHeightDeviation_ * factor;

    eMatrixRot R = matrixRollPitchYaw(step.getRPY());
    eVector3 upperPosition = R * eVector3((1-factor)/2 * step.getSize().x(),0.0, 0.0) + step.getPosition();
    eVector3 lowerPosition = R * eVector3(-(1-factor)/2 * step.getSize().x(),0.0, 0.0)+ step.getPosition();

    std::vector<eVector3> movedPositions;
    movedPositions.push_back(upperPosition);
    movedPositions.push_back(lowerPosition);
    eVector3 sizeReduced3D = eVector3(factor * step.getSize().x(), step.getSize().y() + 2.0 * cellSize_.y(), step.getSize().z());
    std::vector<eVector3>::iterator positionStep = movedPositions.begin();

    for(; positionStep != movedPositions.end(); positionStep++)
    {
        grid_map::Polygon polygon = plannerUtils::getStepPolygon(step.getRPY().z(),eVector2(positionStep->x(), positionStep->y()), eVector2(sizeReduced3D.x(), sizeReduced3D.y()));
        bool isSucces;
        grid_map::GridMap subMap = this->orientedSubMap(polygon, isSucces);
        if(!isSucces)
        {
            return 0;
        }
        grid_map::Matrix flippedPlane = plannerUtils::flipPlane(step.getRPY(), *positionStep, subMap, "elevation");

        if(std::abs(flippedPlane.maxCoeff()) > heightError
                || std::abs(flippedPlane.minCoeff()) > heightError)
        {
            return 0;
        }

    }



    return 1;

}


bool StepValidation::checkStanceLegClearance(const grid_map::Polygon &polygon)
{
    std::vector<grid_map::Position> vertices = stanceLeg_.getVertices();

    for( std::vector<grid_map::Position>::iterator vertex(vertices.begin()); vertex != vertices.end(); ++vertex)
    {
        if(polygon.isInside(*vertex))
        {
            return 0;
        }

    }

    return 1;

}


int StepValidation::getDifferentPlanes(const grid_map::GridMap &subMap) const
{
    int nrRolls{0};
    int nrPitch{0};

    plannerUtils::getNumberOfDifferentValues(subMap.get("roll"), nrRolls);


    plannerUtils::getNumberOfDifferentValues(subMap.get("pitch"), nrPitch);

    return std::max(nrRolls,nrPitch);
}

void StepValidation::getMaxHeightIndex(const grid_map::GridMap &map, const grid_map::Polygon &polygon, grid_map::Index &index) const
{
    grid_map::GridMap::Matrix heightMap = map.get("elevation");

    plannerUtils::getMaxIndex(heightMap,index);

    grid_map::Position position;
    map.getPosition(index,position);


    while(!polygon.isInside(position))
    {
        heightMap(index.x(),index.y()) = std::nan("1");
        plannerUtils::getMaxIndex(heightMap,index);
        map.getPosition(index,position);
    }


}


Footstep6D StepValidation::findClosestStep(const Footstep6D &step)
{
    if(step.isValid())
    {
        return step;
    }

    FootstepArray stepArray;



    bool isSucces;
    visualization_msgs::MarkerArray markerArray;
    grid_map::SubmapGeometry submap(*map_,step.get2DPosition(), closestStepSearchArea_, isSucces);
    for (grid_map::SubmapIterator iterator(submap); !iterator.isPastEnd(); ++iterator)
    {

        eVector3 size(0.2, 0.1, 0.05);
        grid_map::Index index = *iterator;
        eVector2 position;
        map_->getPosition(index,position);
        eVector3 orientation(0.0,0.0,step.getYaw());
        eVector3 stepPosition = eVector3(position.x(),position.y(), 0.0);
        Footstep6D newStep(stepPosition,orientation,step.getSize(),step.getSide());


        stepArray.push_back(newStep);





    }

    //    expandFullMap_ = true;
    //    nodeExpansion_ = true;
    stepArray = this->getValidatedSteps(stepArray, false);
    //    expandFullMap_ = false;
    //    nodeExpansion_ = false;


    FootstepArray::iterator newStep;
    FootstepArray::iterator stepIter = stepArray.begin();
    grid_map::Length footSize = eVector2(step.getSize().x(),
                                         step.getSize().y());

    double minDistance = maxStepTranslation_*1.5;
    for(; stepIter !=stepArray.end(); stepIter++)
    {
        grid_map::Polygon polygon = plannerUtils::getStepPolygon(stepIter->getYaw(),stepIter->get2DPosition(), footSize);

        if(!this->checkStanceLegClearance(polygon))
        {
            stepIter->validateStep(false);
        }


        visualization_msgs::Marker marker;
        stepIter->stepToMarker(marker,"/odom", "node_expansion");
        marker.scale.x = 0.2 * marker.scale.x;
        marker.scale.y = 0.2 * marker.scale.y;
        marker.scale.z = 0.2 * marker.scale.z;


        markerArray.markers.push_back(marker);

        if(!stepIter->isValid())
        {
            continue;
        }
        eVector2 previousStepPosition = stanceLeg_.get2DPosition();
        eVector2 stepTranslation = (previousStepPosition - stepIter->get2DPosition());
        eVector2 stepDistanceVector = step.get2DPosition() - stepIter->get2DPosition();
        if((stepDistanceVector.norm() < minDistance) &&  (stepTranslation.norm() < maxStepTranslation_ * 1.5))
        {
            minDistance = stepDistanceVector.norm();
            newStep = stepIter;
        }

    }

    visualization_msgs::Marker markerList;
    this->stepArrayToMarkerList(stepArray, markerList, "/odom");
    plannerUtils::setMarkerId(markerArray);
    if(nodeExpansion_)
    {
        this->pubStepExpansion(markerList, markerArray);
    }


    if(minDistance < maxStepTranslation_  *1.5)
    {
        std::cout << "Found a valid step close to the original one" << '\n';
        return *newStep;
    }else
    {
        std::cout << "Found no valid step" << '\n';
        return step;
    }


}


boost::shared_ptr<Footstep6D> StepValidation::getPreviousStep(const FootstepArray &footstepArray, FootstepArray::iterator &step) const
{

    boost::shared_ptr<Footstep6D> previousStep;
    if(step == footstepArray.begin())
    {
        std::string sourceFrame;
        switch(step->getSide()._value)
        {
        case pal_locomotion::Side::LEFT:
        {
            sourceFrame = "right_sole_link";
            break;
        }
        case pal_locomotion::Side::RIGHT:
        {
            sourceFrame = "left_sole_link";
            break;
        }
        default:
        {
            sourceFrame = "base_footprint";
        }
        }
        eMatrixHom startPose = pal::getTransform("odom", sourceFrame, ros::Duration(1.0));
        previousStep = boost::make_shared<Footstep6D>(Footstep6D(startPose, footSize_));
    }
    else
    {
        FootstepArray::iterator previousIt = std::prev(step,1);
        previousStep = boost::make_shared<Footstep6D>(*previousIt);
    }

    return previousStep;

}

grid_map::GridMap StepValidation::orientedSubMap(const grid_map::Polygon &polygon, bool &isSucces) const
{

    grid_map::Position positionPoly;
    grid_map::Length lengthPoly;


    polygon.getBoundingBox(positionPoly, lengthPoly);

    geometry_msgs::PolygonStamped polStamped;


    grid_map::GridMap submap =  map_->getSubmap(positionPoly, lengthPoly, isSucces);



    return submap;
}

bool StepValidation::checkIncline(const Footstep6D &step) const
{
    eVector3 rpy = nextStep_.getOrientation();
    if(((std::abs(rpy.x()) >= maxIncline_) && (std::abs(rpy.x()) <= (M_PI-maxIncline_)))
            || ((std::abs(rpy.y()) >= maxIncline_) && (std::abs(rpy.y()) <= (M_PI-maxIncline_))))
    {
        return 0;
    }


    return 1;
}


bool StepValidation::checkHeight(const Footstep6D &nextStep) const
{

    float heightDifference = std::abs(nextStep_.getHeight()-stanceLeg_.getHeight());
    if(heightDifference >= maxHeight_)
    {
        return 0;
    }

    return 1;

}

void StepValidation::pubStep(Footstep6D &step) const
{

    visualization_msgs::Marker marker;
    marker.id =1;
    step.stepToMarker(marker, map_->getFrameId(), "footstep");
    plannerUtils::setMarkerColor(marker,1,1,0);
    pubStep_.publish(marker);


}

void StepValidation::pubStepExpansion(const visualization_msgs::Marker &marker, const visualization_msgs::MarkerArray &markerArray) const
{

    pubStepExpansionList_.publish(marker);
    pubStepExpansionArray_.publish(markerArray);


}




void StepValidation::stepArrayToMarkerList(const FootstepArray &stepArray, visualization_msgs::Marker &marker, const std::string &headerFrame) const
{
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = headerFrame;
    marker.ns = "node_expansion";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 1.0f;

    Eigen::Quaterniond q;
    q.setIdentity();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    for(FootstepArray::const_iterator step = stepArray.begin(); step != stepArray.end(); step++)
    {

        geometry_msgs::Point point;
        point.x = step->getPosition().x();
        point.y = step->getPosition().y();
        point.z = step->getPosition().z();
        marker.points.push_back(point);

        std_msgs::ColorRGBA color;
        color.r = 0.0f;
        color.a = 1.0f;
        color.b = 0.0f;
        color.g = 0.0f;

        if(!step->isValid())
        {
            color.r = 1;
        }else
        {
            color.g = 1;
        }
        marker.colors.push_back(color);

    }


}




void StepValidation::publishMarker(const FootstepArray &stepArray)
{

    footstepMarkerArray_.markers.clear();

    for(int ii{0}; ii < stepArray.size(); ii++)
    {
        visualization_msgs::Marker marker;
        stepArray[ii].stepToMarker(marker, "/odom", "footsteps");
        footstepMarkerArray_.markers.push_back(marker);
    }

    plannerUtils::setMarkerId(footstepMarkerArray_);
    pubMarker_.publish(footstepMarkerArray_);

}

