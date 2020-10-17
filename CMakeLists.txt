cmake_minimum_required(VERSION 2.8.3)

project(static_step_validation)


set(CMAKE_CXX_STANDARD 11)
set(CATKIN_COMPONENTS
    roscpp
    pal_locomotion
    pal_ros_utils
    math_utils
    std_msgs
    grid_map_core
    grid_map_ros
    grid_map_msgs
    message_filters
    custom_planner_msgs
    )

find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
    ${CATKIN_COMPONENTS} message_generation


)
find_package(PCL 1.8 REQUIRED)
set(CMAKE_CXX_FLAGS "-Wno-error=comment -Wno-error=shadow")

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES  ${PROJECT_NAME}
    CATKIN_DEPENDS ${CATKIN_COMPONENTS}
    DEPENDS
     Boost
)

add_library( ${PROJECT_NAME}
  src/StepValidation.cpp
  src/planner_utils.cpp
  src/Footstep6D.cpp
  src/FootWiggle.cpp
  )


include_directories(
      SYSTEM
      include
      ${catkin_INCLUDE_DIRS}
      ${Boost_INCLUDE_DIRS}
      ${PCL_INCLUDE_DIRS}

)


add_executable(step_validation_node
          src/step_validation_node.cpp
          src/StepValidation.cpp
          src/planner_utils.cpp
          src/Footstep6D.cpp
          src/FootWiggle.cpp)


target_link_libraries(step_validation_node
                     ${catkin_LIBRARIES}
)

add_executable(step_interpolation_node
          src/step_interpolation_node.cpp
          src/StepValidation.cpp
          src/planner_utils.cpp
          src/Footstep6D.cpp
          src/FootWiggle.cpp
          src/StepInterpolation.cpp)


target_link_libraries(step_interpolation_node
  ${catkin_LIBRARIES}
)


add_executable(step_expansion_node
          src/step_expansion_node.cpp
          src/StepValidation.cpp
          src/planner_utils.cpp
          src/Footstep6D.cpp
          src/FootWiggle.cpp)



target_link_libraries(step_expansion_node
  ${catkin_LIBRARIES}

)


