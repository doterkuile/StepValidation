#ifndef StepValidation_H
#define StepValidation_H
#include "Footstep6D.h"
#include "custom_planner_msgs/polygonArray.h"
#include "FootWiggle.h"
#include "planner_utils.h"

#include <pal_ros_utils/conversions.h>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/SetGridMap.h>
#include <grid_map_msgs/GetGridMap.h>
#include <Eigen/Geometry>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/time.h>
#include <chrono>

#include <pal_locomotion/static_step_executor.h>
#include <pal_ros_utils/tf_utils.h>
namespace StepValidator{
class StanceLeg
{
public:

    void setStep(const Footstep6D &step);

    double getHeight() const;
    Footstep6D getStep() const;
    std::vector<grid_map::Position> getVertices();
    grid_map::Position get2DPosition() const;

private:

    Footstep6D step_;
    std::vector<grid_map::Position> vertices_;

};

class NextStep
{
public:
    NextStep();
    void setStep(const Footstep6D &step);
    Footstep6D getStep();
    void setStepPolygon(const grid_map::Polygon &polygon);
    grid_map::Polygon getStepPolygon();
    grid_map::Polygon getSidePolygon();
    grid_map::Polygon getFrontPolygon();
    grid_map::Polygon getEnlargedStepPolygon(const Eigen::VectorXd cellSize);
    double getHeight() const;
    eVector3 getPosition() const;
    eVector2 get2DPosition() const;
    grid_map::GridMap getSubMap();
    void setMaxHeightIndex(const grid_map::Index &maxIndex);
    grid_map::Index getMaxHeightIndex();
    eVector3 getOrientation() const;
    void setSubmap(const grid_map::GridMap &submap);
    grid_map::GridMap getSubmap();
    void validateStep(const bool &isValid);
    void hasValidStanceLeg(const bool &isValid);


private:
    Footstep6D step_;
    grid_map::Index maxHeightIndex_;
    grid_map::Polygon stepPolygon_;
    grid_map::Polygon sidePolygon_;
    grid_map::Polygon frontPolygon_;
    grid_map::Polygon enlargedPolygon_;
    grid_map::GridMap submap_;
    bool hasStepPolygon_;
    bool hasSidePolygon_;
    bool hasFrontPolygon_;
    bool hasEnlargedPolygon_;
    bool hasSubmap_;
    bool hasMaxHeightIndex_;
    bool validStep_;
    bool validStanceLeg_;







};

} // End of namespace


class StepValidation
{
public:
    // Basic constructor, retrieves map data through callback
    StepValidation(ros::NodeHandle &nh);

    // Constructor with mapdata
    StepValidation(ros::NodeHandle &nh, const boost::shared_ptr<grid_map::GridMap> &gridMap);

    // Constructor with footstepArray, retrieves map data through callback
    StepValidation(ros::NodeHandle &nh, const FootstepArray &stepArray);

    // Set step array
    void setStepArray(FootstepArray &stepArray);

    // Snap and validate single step, nextFoot, return true if succesfull
    bool getValidatedSteps(Footstep6D &stanceFoot, Footstep6D &nextFoot);

    // Return snapped and validated footstep array
    FootstepArray getValidatedSteps(const bool &findClosestValidStep);

    // Return snapped and validated footstep array
    FootstepArray getValidatedSteps(FootstepArray &stepArray, const bool &findClosestValidStep);

    // Get grid map pointer
    boost::shared_ptr<grid_map::GridMap> getGridMap() const;
    // Update gridmap
    void updateMap(const boost::shared_ptr<grid_map::GridMap> &gridMap);

    // Snap step to environment
    void snapStepToEnvironment(Footstep6D &step) const;

    // Check if class is subscribed to grid map
    bool isSubscribed();

    // publish marker array corresponding to stepArray
    void publishMarker(const FootstepArray &stepArray);

    // Find closest valid step to a given invalid step
    Footstep6D findClosestStep(const Footstep6D &step);

    // Create marker list corresponding to the stepArray
    void stepArrayToMarkerList(const FootstepArray &stepArray, visualization_msgs::Marker &marker, const std::string &headerFrame) const;

    // Return waypoint between current footposition and next position
    eVector3 getWayPoint(const Footstep6D &currentStep, const Footstep6D &nextStep) const;

    // Set Stance leg
    void setStanceLeg(Footstep6D &stanceLeg);
    void resetStanceLeg(const bool resetStanceLeg);

    void setNewStep(Footstep6D &nextStep);
    void resetNextStep(const bool resetNextStep);


private:

    // Setup paramaters of the class
    void setupParams();

    // Setup publishers of the class
    void setupPublishers();

    // Setup subscriber
    void setSubscriber(const std::string &s);

    // Callback for grid map message
    void mapCallBack(const grid_map_msgs::GridMap &inputMsg);

    // Callback for plane polygons (unused)
    void polygonCallBack(const custom_planner_msgs::polygonArray &polygonArray);

    // Publish stepExpansion marker array and marker list
    void pubStepExpansion(const visualization_msgs::Marker &marker, const visualization_msgs::MarkerArray &markerArray) const;

    // Get previous step of footstep array
    boost::shared_ptr<Footstep6D> getPreviousStep(const FootstepArray &footstepArray, FootstepArray::iterator &step) const;

    // Publish marker of a single step
    void pubStep(Footstep6D &step) const;

    // Snap and validate the next step
    void snapAndValidate(Footstep6D &stanceFoot, Footstep6D &nextFoot, const bool &findClosestValidStep);

    // Snap and validate an array of steps
    FootstepArray snapAndValidate(FootstepArray &stepArray, const bool &findClosestValidStep);

    // Validate the currentstep
    void validateStep(const StepValidator::StanceLeg &stanceLeg, StepValidator::NextStep &nextStep);

    // Check height difference between steps
    bool checkHeight(const Footstep6D &nextStep) const;

    // Check incline of step
    bool checkIncline(const Footstep6D &Step) const;

    // Return index of maximum height of the map
    void getMaxHeightIndex(const grid_map::GridMap &map, const grid_map::Polygon &polygon, grid_map::Index &index) const;

    // Return number of different planes of a given grid map
    int getDifferentPlanes(const grid_map::GridMap &subMap) const;

    // Return oriented submap based on a polygon
    grid_map::GridMap orientedSubMap(const grid_map::Polygon &polygon, bool &isSucces) const;

    // Snap step to environment
    void nodeSnapping(Footstep6D &step, const grid_map::Index &maxHeightIndex, const grid_map::GridMap &subMap) const;

    // return true if shin does not collide with obstacle
    bool checkShinCollision(const Footstep6D &step, grid_map::Polygon &frontPolygon, grid_map::Polygon &sidePolygon) const;

    // return true if foot does not collide with obstacles
    bool checkStepCollision(const Footstep6D &step, grid_map::Polygon &polygon) const;

    // Returns true if step has enough foothold
    bool checkPartialFoothold(const grid_map::Polygon &polygon, const Footstep6D &step, const grid_map::GridMap &submap, const grid_map::Index &maxHeightIndex) const;

    // Returns true if step does not collide with stance leg
    bool checkStanceLegClearance(const grid_map::Polygon &stepPolygon);

    // Slightly move step pose to increase foothold (unused)
    void increaseFoothold(Footstep6D &step, const grid_map::Polygon &polygon);







    mutable boost::recursive_mutex mutexMap_;
    ros::NodeHandle nh_;
    ros::Subscriber subMap_;
    ros::Subscriber subPolygonArray_;
    bool setSubs_;
    bool nodeExpansion_;
    bool expandFullMap_;
    bool receivedMap_ = false;
    ros::Publisher pubMap_;
    ros::Publisher pubStep_;
    ros::Publisher pubMarker_;
    ros::Publisher pubGridMap_;
    ros::Publisher pubPolygon_;
    ros::Publisher pubWiggledPolygon_;
    ros::Publisher pubStepExpansionList_;
    ros::Publisher pubStepExpansionArray_;
    visualization_msgs::MarkerArray footstepMarkerArray_;
    XmlRpc::XmlRpcValue config_;
    boost::shared_ptr<grid_map::GridMap> map_;
    Eigen::Vector2d cellSize_;
    FootstepArray footstepArray_;
    FootstepArray validatedStepArray_;
    eVector3 footSize_;
    double maxStepTranslation_;
    custom_planner_msgs::polygonArray polygonArray_;

    double maxHeight_;
    double shinCollisionHeight_;
    double stepCollisionHeight_;
    double maxIncline_;
    double minFoothold_;
    double partialFootholdHeightDeviation_;
    eVector2 closestStepSearchArea_;

    // Avoid resnapping of stance leg or next step in case of validating primitives
    bool resetStanceLeg_;
    bool resetStep_;

    StepValidator::NextStep nextStep_;
    StepValidator::StanceLeg stanceLeg_;

};

//}



#endif // StepValidation_H
