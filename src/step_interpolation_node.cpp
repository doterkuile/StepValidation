#include "StepInterpolation.h"


void createBipedStepList(ros::NodeHandle &nh,FootstepArray &stepArray, StepValidation &stepValidator)
{

    ros::Duration switch_duration(0.3);
    ros::Duration stance_duration(0.9);
    bool executeSteps;
    nh.getParam("/execute_steps",executeSteps);
  //  eMatrixHom global_transform = pal::getTransform("odom", "base_footprint", ros::Duration(1.0));

  //  eMatrixHom globalLeft = pal::getTransform("odom", "left_sole_link", ros::Duration(1.0));
  //  eMatrixHom globalRight = pal::getTransform("odom", "right_sole_link", ros::Duration(1.0));

    eMatrixHom footTransform;
    eMatrixHom footPose;
    eMatrixHom footShift;

  //  eMatrixHom globalRight;

      eMatrixHom global_transform;

    BipedStepList sl;

    pal_locomotion::Side side;
    FootstepArray::iterator executedStep = stepArray.begin();

        FootstepArray::iterator step = stepArray.begin();

        global_transform = pal::getTransform("odom", "base_footprint", ros::Duration(1.0));
     ROS_INFO_STREAM("excuting steps...");
    for(; step !=stepArray.end(); step++)
    {
        ROS_INFO_STREAM("add step to bipesteplist");
        boost::shared_ptr<Footstep6D> previousStep;



        std::string sourceFrame;
        if(step->getSide()._value == pal_locomotion::Side::RIGHT)
        {
          sourceFrame = "right_sole_link";
          footTransform = pal::getTransform("odom", "left_sole_link", ros::Duration(1.0));
        }
        else if(step->getSide()._value == pal_locomotion::Side::LEFT)
        {
            sourceFrame = "left_sole_link";
            footTransform =  pal::getTransform("odom", "right_sole_link", ros::Duration(1.0));


        } else
        {
            sourceFrame = "base_footprint";
            footTransform.setIdentity();

        }
        eMatrixHom startPose = pal::getTransform("odom", sourceFrame, ros::Duration(1.0));
        std::cout << "startPose = " << startPose.translation() << '\n';


        if(step == stepArray.begin())
        {

            previousStep = boost::make_shared<Footstep6D>(Footstep6D(startPose,step->getSize()));
        }
        else
        {
           FootstepArray::iterator previousIt = std::prev(step,1);
           previousStep = boost::make_shared<Footstep6D>(*previousIt);
           footPose = previousStep->getPose();
           footShift =footTransform.inverse() * footPose;
           global_transform = footShift * global_transform;

        }

        if(!step->isValid())
        {
            std::cout << "step is not valid finding new step" << '\n';
//            *step = stepValidator.findClosestStep(*previousStep, *step);
        }


        sl.clear();

      eQuaternion qTemp = quaternionRollPitchYaw(step->getRPY());
      eMatrixHom step_pose = HommatrixRollPitchYaw(step->getRPY().x(), step->getRPY().y(), step->getRPY().z());
      step_pose.translate(step->getPosition());
      bool allStepsValid =true;
      for(int ii{0}; ii < stepArray.size(); ii++)
      {
          if(!stepArray[ii].isValid())
          {
              allStepsValid = false;
          }

      }

      if(executeSteps && allStepsValid && (executedStep == step))
      {
          executedStep++;
          std::cout << "executing the " << step->getSide() << "step " << '\n';
          std::cout << "to position " << step->getPosition() << '\n';
          std::cout << "and orientation " << step->getRPY() << '\n';



          Step bipedStep(switch_duration, stance_duration, global_transform * step_pose, step->getSide());
          sl.push_back(bipedStep);
          StaticStepExecutor executor(nh, ros::Duration(1.0),
                                      ros::Duration(1.0), sl);

          executor.execute();
      }
    }


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "step_interpolation_node");
  ros::NodeHandle nh("~");

  ROS_INFO_STREAM(">>> Starting node, new");
  StepInterpolation stepInterpolation(nh);
  FootstepArray footstepArray;
  stepInterpolation.getStepArray(footstepArray);
  StepValidation stepValidator(nh);
  stepValidator.setStepArray(footstepArray);
  bool executeSteps;
  nh.getParam("/execute_steps",executeSteps);
  FootstepArray validatedSteps;
  boost::mutex mutex;

  ros::AsyncSpinner spinner(0);

  spinner.start();

// ros::Rate rate(10);
  while(ros::ok())
  {
    boost::lock_guard<boost::mutex> guard(mutex);

    validatedSteps = stepValidator.getValidatedSteps(true);
    std::cout << "got validated steps" << '\n';
    stepInterpolation.publishGoalPose();
    stepInterpolation.publishInterpolationSteps();

    stepValidator.publishMarker(validatedSteps);

    if(stepValidator.isSubscribed())
    {
//       createBipedStepList(nh, validatedSteps, stepValidator);

    }

//  rate.sleep();
  }
  ros::waitForShutdown();

//  ros::spin();

  return 0;
}
