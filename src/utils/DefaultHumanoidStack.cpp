#include <OpenSoT/utils/DefaultHumanoidStack.h>


using namespace OpenSoT;

DefaultHumanoidStack::DefaultHumanoidStack(iDynUtils& model,
                                           const double dT,
                                           const yarp::sig::Vector& state) :
     leftArm( new tasks::velocity::Cartesian("cartesian::l_wrist",
                                             state,
                                             model,
                                             model.left_arm.end_effector_name,
                                             "world") ),
     leftArm_Position( new SubTask(leftArm, SubTask::SubTaskMap::range(0,2)) ),
     leftArm_Orientation( new SubTask(leftArm, SubTask::SubTaskMap::range(3,5)) ),
     rightArm( new tasks::velocity::Cartesian("cartesian::r_wrist",
                                              state,
                                              model,
                                              model.right_arm.end_effector_name,
                                             "world") ),
     rightArm_Position( new SubTask(rightArm, SubTask::SubTaskMap::range(0,2)) ),
     rightArm_Orientation( new SubTask(rightArm, SubTask::SubTaskMap::range(3,5)) ),
     waist2LeftArm( new tasks::velocity::Cartesian("cartesian::l_wrist",
                                                   state,
                                                   model,\
                                                   model.left_arm.end_effector_name,
                                                   "Waist") ),
     waist2LeftArm_Position( new SubTask(waist2LeftArm, SubTask::SubTaskMap::range(0,2)) ),
     waist2LeftArm_Orientation( new SubTask(waist2LeftArm, SubTask::SubTaskMap::range(3,5)) ),
     waist2RightArm( new tasks::velocity::Cartesian("cartesian::r_wrist",
                                                    state,
                                                    model,
                                                    model.right_arm.end_effector_name,
                                                    "Waist") ),
     waist2RightArm_Position( new SubTask(waist2RightArm, SubTask::SubTaskMap::range(0,2)) ),
     waist2RightArm_Orientation( new SubTask(waist2RightArm, SubTask::SubTaskMap::range(3,5)) ),
     leftLeg( new tasks::velocity::Cartesian("cartesian::l_sole",
                                             state,
                                             model,
                                             model.left_leg.end_effector_name,
                                             "world") ),
     leftLeg_Position( new SubTask(leftLeg, SubTask::SubTaskMap::range(0,2)) ),
     leftLeg_Orientation( new SubTask(leftLeg, SubTask::SubTaskMap::range(3,5)) ),
     rightLeg( new tasks::velocity::Cartesian("cartesian::r_sole",
                                              state,
                                              model,
                                              model.right_leg.end_effector_name,
                                              "world") ),
     rightLeg_Position( new SubTask(rightLeg, SubTask::SubTaskMap::range(0,2)) ),
     rightLeg_Orientation( new SubTask(rightLeg, SubTask::SubTaskMap::range(3,5)) ),
     waist( new tasks::velocity::Cartesian("cartesian::waist",
                                            state,
                                            model,
                                            "Waist",
                                            "world") ),
     waist_Position( new SubTask(waist, SubTask::SubTaskMap::range(0,2)) ),
     waist_Position_XY( new SubTask(waist, SubTask::SubTaskMap::range(0,1)) ),
     waist_Position_Z( new SubTask(waist, SubTask::SubTaskMap::range(2,2)) ),
     waist_Orientation( new SubTask(waist, SubTask::SubTaskMap::range(3,5)) ),
     com( new tasks::velocity::CoM(state,
                                   model) ),
     com_XY( new SubTask(com, SubTask::SubTaskMap::range(0,1)) ),
     com_Z( new SubTask(com, SubTask::SubTaskMap::range(2,2)) ),
     postural( new tasks::velocity::Postural(state) ),
     comVelocity( new constraints::velocity::CoMVelocity(yarp::sig::Vector(3,.3),
                                                         dT,
                                                         state,
                                                         model) ),
     convexHull( new constraints::velocity::ConvexHull(state,
                                                       model) ),
     jointLimits( new constraints::velocity::JointLimits(state,
                                                         model.iDyn3_model.getJointBoundMax(),
                                                         model.iDyn3_model.getJointBoundMin()) ),
     velocityLimits( new constraints::velocity::VelocityLimits(0.3,
                                                               dT,
                                                               state.size()) )


 {

 }


