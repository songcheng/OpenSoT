/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Enrico Mingo, Alessio Rocchi
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "sot_VelKinCon_ctrl.h"
#include <boost/foreach.hpp>
#include "task_solver.h"
#include <drc_shared/cartesian_utils.h>
#include "sot_VelKinCon_constants.h"
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>

#define toRad(X) (X*M_PI/180.0)
#define toDeg(X) (X*180.0/M_PI)
#define MilliSecToSec(X) (X/1000.0)

/** ******************************************* **/

using namespace iCub::iDynTree;
using namespace yarp::math;
using namespace yarp::os;
using namespace wb_sot;

sot_VelKinCon_ctrl::sot_VelKinCon_ctrl(const int period,    const bool _LEFT_ARM_IMPEDANCE,
                                                            const bool _RIGHT_ARM_IMPEDANCE,
                                                            const bool _TORSO_IMPEDANCE,
                                                            paramHelp::ParamHelperServer* _ph):
    RateThread(period),
    IYarp(),
    q_ref(1),
    dq_ref(1),
    ddq_ref(1),
    tau_gravity(1),
    q(1),
    q_left_arm(1),
    q_right_arm(1),
    q_left_leg(1),
    q_right_leg(1),
    q_torso(1),
    right_arm_pos_ref(3, 0.0),
    left_arm_pos_ref(3, 0.0),
    worldT(4,4),
    t_elapsed(0.0),
    gradientGq(1,0.0),
    eRWrist_p(3, 0.0),
    eRWrist_o(3, 0.0),
    eLWrist_p(3, 0.0),
    eLWrist_o(3, 0.0),
    eSwingFoot_p(3, 0.0),
    eSwingFoot_o(3, 0.0),
    eCoM(3, 0.0),
    LEFT_ARM_IMPEDANCE(_LEFT_ARM_IMPEDANCE),
    RIGHT_ARM_IMPEDANCE(_RIGHT_ARM_IMPEDANCE),
    TORSO_IMPEDANCE(_TORSO_IMPEDANCE),
    paramHelper(_ph)
{
    int nJ = idynutils.coman_iDyn3.getNrOfDOFs();

    gradientGq.resize(nJ);
    Q_postural.resize(nJ, nJ);
    Q_postural.eye();
//    yarp::sig::Vector qMax = coman_iDyn3.getJointBoundMax();
//    yarp::sig::Vector qMin = coman_iDyn3.getJointBoundMin();
//    Q_postural.diagonal(computeW(qMin, qMax, right_arm_joint_numbers,
//                                 left_arm_joint_numbers, waist_joint_numbers));

    zero.resize(1);
    zero.zero();

    q.resize(nJ, 0.0);
    q_ref.resize(nJ, 0.0);
    dq_ref.resize(nJ,0.0);
    ddq_ref.resize(nJ, 0.0);

    tau_gravity.resize(nJ, 0.0);

    q_left_arm.resize(IYarp.left_arm.getNumberOfJoints(), 0.0);
    q_right_arm.resize(IYarp.right_arm.getNumberOfJoints(), 0.0);
    q_left_leg.resize(IYarp.left_leg.getNumberOfJoints(), 0.0);
    q_right_leg.resize(IYarp.right_leg.getNumberOfJoints(), 0.0);
    q_torso.resize(IYarp.torso.getNumberOfJoints(), 0.0);

    worldT.eye();

    is_clik = false;
}

/** @todo move to drc_shared */
void sot_VelKinCon_ctrl::parameterUpdated(const ParamProxyInterface *pd)
{
    return;
}

/** @todo move to drc_shared */
void sot_VelKinCon_ctrl::commandReceived(const CommandDescription &cd,
                                         const Bottle &params, Bottle &reply)
{
    switch(cd.id)
    {
    case wb_sot::COMMAND_ID_HELP:
        paramHelper->getHelpMessage(reply);
        break;
    case wb_sot::COMMAND_ID_SAVE_PARAMS:
        {
            std::string fileName = CONF_NAME;
            yarp::os::ResourceFinder rf;
            rf.setDefaultContext(MODULE_NAME);
            std::string folderName = rf.getContextPath() + "/";
            std::string confPath = folderName + fileName;
            std::vector<int> configIds;
            for(unsigned int i = 0; i < PARAM_ID_SIZE; ++i)
                if( sot_VelKinCon_ParamDescr[i]->ioType.value == paramHelp::PARAM_IN_OUT ||
                    sot_VelKinCon_ParamDescr[i]->ioType.value == paramHelp::PARAM_CONFIG )
                    configIds.push_back(i);

            std::cout << "Saving to " << confPath;

            std::stringstream ss;
            boost::posix_time::ptime pt = boost::posix_time::second_clock::local_time();
            boost::posix_time::time_facet* output_facet = new boost::posix_time::time_facet("%Y%m%dT%H%M%S%F%q");
            ss.imbue(std::locale(ss.getloc(), output_facet));
            ss << pt;
            std::string confPathWithTimestamp = confPath + "." + ss.str();

            std::cout << " and " << confPathWithTimestamp;
            reply.addString("saving...");

            if( paramHelper->writeParamsOnFile( confPathWithTimestamp,
                                                configIds.data(),
                                                configIds.size())) {
                if(boost::filesystem::exists(confPath))
                           boost::filesystem::remove(confPath);
                paramHelper->writeParamsOnFile( confPath,
                                                configIds.data(),
                                                configIds.size());
                reply.addString("ok");
            } else
                reply.addString("failed!");
        }
        break;
    }
}

//Qui devo prendere la configurazione iniziale del robot!
/** @todo move the paramHelper stuff to the yarp_interface */
bool sot_VelKinCon_ctrl::threadInit()
{
    IYarp.cleanPorts();

    dq_ref.zero();
    getFeedBack();

    //Here we set as initial reference the measured value: this will be the postural task
    q_ref = q;

    updateiDyn3Model(true);

    support_foot_LinkIndex = idynutils.left_leg.index;
    swing_foot_LinkIndex = idynutils.right_leg.index;

    right_arm_pos_ref = idynutils.coman_iDyn3.getPosition(idynutils.right_arm.index);
    left_arm_pos_ref = idynutils.coman_iDyn3.getPosition(idynutils.left_arm.index);

    com_pos_ref = idynutils.coman_iDyn3.getCOM("",support_foot_LinkIndex);

    swing_foot_pos_ref = idynutils.coman_iDyn3.getPosition(support_foot_LinkIndex, swing_foot_LinkIndex);

    ROS_INFO("Initial Pose Ref left_arm:");   cartesian_utils::printHomogeneousTransform(left_arm_pos_ref);std::cout<<std::endl;
    ROS_INFO("Initial Pose Ref right_arm:");  cartesian_utils::printHomogeneousTransform(right_arm_pos_ref);std::cout<<std::endl;
    ROS_INFO("Initial Pose Ref swing_foot:"); cartesian_utils::printHomogeneousTransform(swing_foot_pos_ref);std::cout<<std::endl;
    ROS_INFO("Initial Position Ref CoM: [ %s ]", com_pos_ref.toString().c_str());

if(RIGHT_ARM_IMPEDANCE) {
    ROS_INFO("Setting Impedance Mode for q_right_arm:");
    for(unsigned int i = 0; i < q_right_arm.size(); ++i)
        IYarp.right_arm.controlMode->setImpedancePositionMode(i);
} else {
    ROS_INFO("Setting Position Mode for q_right_arm:");
    for(unsigned int i = 0; i < q_right_arm.size(); ++i)
        IYarp.right_arm.controlMode->setPositionMode(i);
}

if(LEFT_ARM_IMPEDANCE) {
    ROS_INFO("Setting Impedance Mode for q_left_arm:");
    for(unsigned int i = 0; i < q_left_arm.size(); ++i)
        IYarp.left_arm.controlMode->setImpedancePositionMode(i);
} else {
    ROS_INFO("Setting Position Mode for q_left_arm:");
    for(unsigned int i = 0; i < q_left_arm.size(); ++i)
        IYarp.left_arm.controlMode->setPositionMode(i);
}

if(TORSO_IMPEDANCE) {
    ROS_INFO("Setting Impedance Mode for q_torso:");
    for(unsigned int i = 0; i < q_torso.size(); ++i)
        IYarp.torso.controlMode->setImpedancePositionMode(i);
} else {
    ROS_INFO("Setting Position Mode for q_torso:");
    for(unsigned int i = 0; i < q_torso.size(); ++i)
        IYarp.torso.controlMode->setPositionMode(i);
}

    ROS_INFO("Setting Position Mode for q_right_leg:");
    for(unsigned int i = 0; i < q_right_leg.size(); ++i)
        IYarp.right_leg.controlMode->setPositionMode(i);

    ROS_INFO("Setting Position Mode for q_left_leg:");
    for(unsigned int i = 0; i < q_left_leg.size(); ++i)
        IYarp.left_leg.controlMode->setPositionMode(i);

    if(is_clik)
        ROS_WARN("SoT is running as CLIK");
    else
        ROS_WARN("SoT is NOT running as CLIK");

    ROS_INFO("sot_VelKinCon START!!!");

    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_COMPUTATION_TIME,             &t_elapsed));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_LEFT_ARM_POSITION_ERROR,      eLWrist_p.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_LEFT_ARM_ORIENTATION_ERROR,   eLWrist_o.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_RIGHT_ARM_POSITION_ERROR,     eRWrist_p.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_RIGHT_ARM_ORIENTATION_ERROR,  eRWrist_o.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_SWING_FOOT_POSITION_ERROR,    eSwingFoot_p.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_SWING_FOOT_ORIENTATION_ERROR, eSwingFoot_o.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_COM_POSITION_ERROR,           eCoM.data()));

    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_USE_3_STACKS,                 &use_3_stacks));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_MAX_JOINT_VELOCITY,           &max_joint_velocity));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_ORIENTATION_ERROR_GAIN,       &orientation_error_gain));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_LAST_STACK_TYPE,              &last_stack_type));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_POSTURAL_WEIGHT_COEFFICIENT,  &postural_weight_coefficient));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_MINEFFORT_WEIGHT_COEFFICIENT, &mineffort_weight_coefficient));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_W_TORSO_WEIGHT,               &w_torso_weight));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_NWSR0,                &qpOASES_NWSR0));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_NWSR1,                &qpOASES_NWSR1));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_NWSR2,                &qpOASES_NWSR2));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_ENABLEREGULARISATION0,&qpOASES_enableRegularisation0));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_ENABLEREGULARISATION1,&qpOASES_enableRegularisation1));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_ENABLEREGULARISATION2,&qpOASES_enableRegularisation2));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER0,&qpOASES_eps0));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER1,&qpOASES_eps1));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER2,&qpOASES_eps2));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_CLIK,                                &is_clik));

    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_HELP,           this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_SAVE_PARAMS,    this));

    return true;
}

void sot_VelKinCon_ctrl::run()
{
#ifdef DEBUG
    paramHelper->lock();
    paramHelper->readStreamParams();
#endif

    IYarp.tic();

    checkInput();

    if(is_clik)
        getFeedBack();
    else
        q += dq_ref;


    updateiDyn3Model(true);

    if(controlLaw())
        move();

    IYarp.sendWorldToBaseLinkPose(idynutils.coman_iDyn3.getWorldBasePose());
    IYarp.sendLSoleToCoMPose(idynutils.coman_iDyn3.getCOM());

    t_elapsed = IYarp.toc();

#ifdef DEBUG
    paramHelper->sendStreamParams();
    paramHelper->unlock();
#endif
}

yarp::sig::Vector sot_VelKinCon_ctrl::computeW(const yarp::sig::Vector &qMin,
                                               const yarp::sig::Vector &qMax,
                                               const std::vector<unsigned int>& right_arm_joint_numbers,
                                               const std::vector<unsigned int>& left_arm_joint_numbers,
                                               const std::vector<unsigned int>& waist_joint_numbers,
                                               const double w_torso_weight)
{
    yarp::sig::Vector w(qMax.size(), 1.0);

    std::vector<unsigned int> waist_left_arm_joint_numbers = waist_joint_numbers;
    std::vector<unsigned int> waist_right_arm_joint_numbers = waist_joint_numbers;
    waist_left_arm_joint_numbers.insert(waist_left_arm_joint_numbers.end(), left_arm_joint_numbers.begin(), left_arm_joint_numbers.end());
    waist_right_arm_joint_numbers.insert(waist_right_arm_joint_numbers.end(), right_arm_joint_numbers.begin(), right_arm_joint_numbers.end());

    for(unsigned int i = 0; i < waist_left_arm_joint_numbers.size(); ++i)
        w[waist_left_arm_joint_numbers[i]]  *= (double)(waist_left_arm_joint_numbers.size() - i);

    for(unsigned int i = 0; i < waist_right_arm_joint_numbers.size(); ++i)
        w[waist_right_arm_joint_numbers[i]] *= (double)(waist_right_arm_joint_numbers.size() - i);

    for(unsigned int i = 0; i < waist_joint_numbers.size(); ++i) {
        w[waist_joint_numbers[i]] = sqrt(w[waist_joint_numbers[i]]);
        w[waist_joint_numbers[i]] *= w_torso_weight;
    }

    w = w/(qMax-qMin);
    for(unsigned int j = 0; j < w.size(); ++j)
        w[j] = fabs(w[j]);

    ROS_INFO("W: [ %s ]", w.toString().c_str());
    return w;
}

void sot_VelKinCon_ctrl::updateiDyn3Model(const bool set_world_pose)
{
    static yarp::sig::Vector zeroes(q.size(),0.0);
    
    idynutils.updateiDyn3Model(q,zeroes,zeroes);
if (LEFT_ARM_IMPEDANCE || RIGHT_ARM_IMPEDANCE || TORSO_IMPEDANCE) {
    idynutils.coman_iDyn3.dynamicRNEA();
    tau_gravity = idynutils.coman_iDyn3.getTorques();
}

    // Set World Pose we do it at the beginning
    if(set_world_pose)
    {
        idynutils.setWorldPose();
    }
}

//Also here the configurations come in deg so we need to convert to rad!
void sot_VelKinCon_ctrl::getFeedBack()
{
    IYarp.left_arm.sense(q_left_arm);
    IYarp.right_arm.sense(q_right_arm);
    IYarp.left_leg.sense(q_left_leg);
    IYarp.right_leg.sense(q_right_leg);
    IYarp.torso.sense(q_torso);

    //To make things faster: we suppose that arms has same number of dofs
    for(unsigned int i = 0; i < q_left_arm.size(); ++i)
    {
        q_left_arm[i] = toRad(q_left_arm[i]); //from deg to rad!
        q_right_arm[i] = toRad(q_right_arm[i]); //from deg to rad!
    }
    //To make things faster: we suppose that legs has same number of dofs
    for(unsigned int i = 0; i < q_left_leg.size(); ++i)
    {
        q_left_leg[i] = toRad(q_left_leg[i]); //from deg to rad!
        q_right_leg[i] = toRad(q_right_leg[i]); //from deg to rad!
    }
    for(unsigned int i = 0; i < q_torso.size(); ++i)
    {
        q_torso[i] = toRad(q_torso[i]); //from deg to rad!
    }
    idynutils.fromRobotToIDyn(q_left_arm,q,idynutils.left_arm);
    idynutils.fromRobotToIDyn(q_right_arm,q,idynutils.right_arm);
    idynutils.fromRobotToIDyn(q_left_leg,q,idynutils.left_leg);
    idynutils.fromRobotToIDyn(q_right_leg,q,idynutils.right_leg);
    idynutils.fromRobotToIDyn(q_torso,q,idynutils.torso);
    
}

void sot_VelKinCon_ctrl::checkInput()
{
    IYarp.getLeftArmCartesianRef(left_arm_pos_ref);
    IYarp.getRightArmCartesianRef(right_arm_pos_ref);
    IYarp.getCoMCartesianRef(com_pos_ref);
    //IYarp.getSetClik(is_clik);
}

/** Here we convert from rad to deg!
    The implemented control is like a velocity control: q_d = q + dq
**/
void sot_VelKinCon_ctrl::move()
{
    yarp::sig::Vector torso(q_torso.size(), 0.0);
    yarp::sig::Vector left_arm(q_left_arm.size(), 0.0);
    yarp::sig::Vector right_arm(q_right_arm.size(), 0.0);
    yarp::sig::Vector left_leg(q_left_leg.size(), 0.0);
    yarp::sig::Vector right_leg(q_right_leg.size(), 0.0);

    double q_sent = 0.0;
    for(unsigned int i = 0; i < torso.size(); ++i){
        q_sent = q[idynutils.torso.joint_numbers[i]] + dq_ref[idynutils.torso.joint_numbers[i]];
        torso[i] = toDeg( q_sent );}
    //Here we assumes that left and right arm has the same number of joints!
    for(unsigned int i = 0; i < left_arm.size(); ++i){
        q_sent = q[idynutils.left_arm.joint_numbers[i]] + dq_ref[idynutils.left_arm.joint_numbers[i]];
        left_arm[i] = toDeg( q_sent );
        q_sent = q[idynutils.right_arm.joint_numbers[i]] + dq_ref[idynutils.right_arm.joint_numbers[i]];
        right_arm[i] = toDeg( q_sent );
    }
    //Here we assumes that left and right leg has the same number of joints!
    for(unsigned int i = 0; i < left_leg.size(); ++i){
        q_sent = q[idynutils.left_leg.joint_numbers[i]] + dq_ref[idynutils.left_leg.joint_numbers[i]];
        left_leg[i] = toDeg( q_sent );
        q_sent = q[idynutils.right_leg.joint_numbers[i]] + dq_ref[idynutils.right_leg.joint_numbers[i]];
        right_leg[i] = toDeg( q_sent );
    }

    IYarp.torso.move(torso);
    IYarp.left_arm.move(left_arm);
    IYarp.right_arm.move(right_arm);
    IYarp.left_leg.move(left_leg);
    IYarp.right_leg.move(right_leg);

    /// GRAVITY COMPENSATION
    if(LEFT_ARM_IMPEDANCE) {
        yarp::sig::Vector tau_gravity_left_arm = getGravityCompensationTorque(idynutils.left_arm.joint_names);
        for(unsigned int i = 0; i < idynutils.left_arm.joint_names.size(); ++i)
            IYarp.left_arm.impedancePositionControl->setImpedanceOffset(i, tau_gravity_left_arm[i]);
    }
    if(RIGHT_ARM_IMPEDANCE) {
        yarp::sig::Vector tau_gravity_right_arm = getGravityCompensationTorque(idynutils.right_arm.joint_names);
        for(unsigned int i = 0; i < idynutils.right_arm.joint_names.size(); ++i)
            IYarp.right_arm.impedancePositionControl->setImpedanceOffset(i, tau_gravity_right_arm[i]);
    }
    if(TORSO_IMPEDANCE) {
        yarp::sig::Vector tau_gravity_torso = getGravityCompensationTorque(idynutils.torso.joint_names);
        for(unsigned int i = 0; i < idynutils.torso.joint_names.size(); ++i)
            IYarp.torso.impedancePositionControl->setImpedanceOffset(i, tau_gravity_torso[i]);
    }
}

bool sot_VelKinCon_ctrl::controlLaw()
{
    yarp::sig::Matrix pos_wrist_R = idynutils.coman_iDyn3.getPosition(idynutils.right_arm.index);
    yarp::sig::Matrix pos_wrist_L = idynutils.coman_iDyn3.getPosition(idynutils.left_arm.index);

    yarp::sig::Vector pos_CoM = idynutils.coman_iDyn3.getCOM("",support_foot_LinkIndex);

    yarp::sig::Matrix pos_foot_swing = idynutils.coman_iDyn3.getPosition(support_foot_LinkIndex,swing_foot_LinkIndex);

    yarp::sig::Matrix JRWrist;
    if(!idynutils.coman_iDyn3.getJacobian(idynutils.right_arm.index,JRWrist))
        ROS_ERROR("Error computing Jacobian for Right Wrist");
    JRWrist = JRWrist.removeCols(0,6);    // removing unactuated joints (floating base)

    yarp::sig::Matrix JLWrist;
    if(!idynutils.coman_iDyn3.getJacobian(idynutils.left_arm.index,JLWrist))
        ROS_ERROR("Error computing Jacobian for Left Wrist");
    JLWrist = JLWrist.removeCols(0,6);    // removing unactuated joints (floating base)

    yarp::sig::Matrix JSwingFoot; // for now, SwingFoot is Left
    if(!idynutils.coman_iDyn3.getRelativeJacobian(swing_foot_LinkIndex,support_foot_LinkIndex,JSwingFoot,true))
        ROS_ERROR("Error computing Jacobian for Left Foot");

    yarp::sig::Matrix JCoM;
    //
    idynutils.coman_iDyn3.setFloatingBaseLink(support_foot_LinkIndex);
    if(!idynutils.coman_iDyn3.getCOMJacobian(JCoM))
        ROS_ERROR("Error computing CoM Jacobian");
    idynutils.coman_iDyn3.setFloatingBaseLink(idynutils.torso.index);
    JCoM = JCoM.removeCols(0,6);    // remove floating base
    JCoM = JCoM.removeRows(3,3);    // remove orientation

    extractJacobians(JRWrist, JLWrist);

    cartesian_utils::computeCartesianError(pos_wrist_R, right_arm_pos_ref,
                                           eRWrist_p, eRWrist_o);
    cartesian_utils::computeCartesianError(pos_wrist_L, left_arm_pos_ref,
                                           eLWrist_p, eLWrist_o);
    cartesian_utils::computeCartesianError(pos_foot_swing, swing_foot_pos_ref,
                                           eSwingFoot_p, eSwingFoot_o);
    eCoM = com_pos_ref-pos_CoM;

    yarp::sig::Vector eRWrist = yarp::math::cat(eRWrist_p, -orientation_error_gain*eRWrist_o);
    yarp::sig::Vector eLWrist = yarp::math::cat(eLWrist_p, -orientation_error_gain*eLWrist_o);
    yarp::sig::Vector eSwingFoot = yarp::math::cat(eSwingFoot_p, -orientation_error_gain*eSwingFoot_o);

//    std::cout<<"eRWrist: "<<eRWrist.toString()<<std::endl;
//    std::cout<<"eLWrist: "<<eLWrist.toString()<<std::endl;
//    std::cout<<"eSwingFoot: "<<eSwingFoot.toString()<<std::endl;
//    std::cout<<"eCoM: "<<eCoM.toString()<<std::endl;
//    std::cout<<"com_pos_ref: "<<com_pos_ref.toString()<<std::endl;
//    std::cout<<"pos_CoM: "<<pos_CoM.toString()<<std::endl;

yarp::sig::Matrix JEe;
yarp::sig::Vector eEe;

if(use_3_stacks) {
    JEe = yarp::math::pile(JRWrist, JLWrist);
    JEe = yarp::math::pile(JEe, JSwingFoot);
    eEe = yarp::math::cat(eRWrist, eLWrist);
    eEe = yarp::math::cat(eEe, eSwingFoot);
} else {
    JEe = yarp::math::pile(JRWrist, JLWrist);
    JEe = yarp::math::pile(JEe, JSwingFoot);
    JEe = yarp::math::pile(JEe, JCoM);
    eEe = yarp::math::cat(eRWrist, eLWrist);
    eEe = yarp::math::cat(eEe, eSwingFoot);
    eEe = yarp::math::cat(eEe, eCoM);
}

    /** Set of last tasks **/
    /**
    *  Here we stack the last set of tasks:
    *
    *  Qdq + (q_ref - q) <-- Postural task
    *  (-grad(g(q))/tau_max)dq <-- Min effort
    *
    **/
    yarp::sig::Vector eq = (q_ref - q);

    yarp::sig::Matrix W(idynutils.coman_iDyn3.getJointTorqueMax().size(), idynutils.coman_iDyn3.getJointTorqueMax().size());
    W.eye();
    for(unsigned int i = 0; i < idynutils.coman_iDyn3.getJointTorqueMax().size(); ++i)
        W(i,i) = 1.0 / (idynutils.coman_iDyn3.getJointTorqueMax()[i]*idynutils.coman_iDyn3.getJointTorqueMax()[i]);
    gradientGq = -1.0 * getGravityCompensationGradient(W);

    yarp::sig::Matrix F;
    yarp::sig::Vector f;
    qpOASES::HessianType qpOasesPosturalHessianType = qpOASES::HST_UNKNOWN;


    if(last_stack_type == LAST_STACK_TYPE_POSTURAL)
    {
        /**
          *  (dq - e)'Q^2(dq - e)
          **/
        eq.zero();
        F = Q_postural;
        f = Q_postural * eq;
        qpOasesPosturalHessianType = qpOASES::HST_POSDEF;
    }
    else if(last_stack_type == LAST_STACK_TYPE_MINIMUM_EFFORT)
    {
        /**
          *  (dq + grad(g(q)))'(dq + grad(g(q)))
          **/
        unsigned int nJ = idynutils.coman_iDyn3.getNrOfDOFs();
        F.resize(nJ,nJ); F.eye();
        f = mineffort_weight_coefficient * gradientGq;
        qpOasesPosturalHessianType = qpOASES::HST_POSDEF;
    }
    else if(last_stack_type == LAST_STACK_TYPE_POSTURAL_AND_MINIMUM_EFFORT)
    {
        /**
          *  (dq - e)'Q^2(dq - e)
          *  (dq + grad(g(q)))'(dq + grad(g(q)))
          **/
        unsigned int nJ = idynutils.coman_iDyn3.getNrOfDOFs();
        yarp::sig::Matrix I; I.resize(nJ,nJ); I.eye();

        double l = postural_weight_coefficient;
        double l2 = 1.0 - l;
        F = yarp::math::pile( l * Q_postural, l2 * I );
        f = yarp::math::cat( l * Q_postural * eq, l2 * mineffort_weight_coefficient * gradientGq);
        qpOasesPosturalHessianType = qpOASES::HST_POSDEF;
    }


    bool control_computed = false;
    if(use_3_stacks) {
        control_computed = task_solver::computeControlHQP(JCoM,eCoM,
                                                          JEe, eEe,
                                                          F, f, qpOasesPosturalHessianType,
                                                          idynutils.coman_iDyn3.getJointBoundMax(),
                                                          idynutils.coman_iDyn3.getJointBoundMin(),
                                                          q, max_joint_velocity,
                                                          MilliSecToSec(getRate()),
                                                          dq_ref, 0.5);
    } else {
        control_computed = task_solver::computeControlHQP(JEe, eEe,
                                                         F, f, qpOasesPosturalHessianType,
                                                         idynutils.coman_iDyn3.getJointBoundMax(),
                                                         idynutils.coman_iDyn3.getJointBoundMin(),
                                                         q, max_joint_velocity,
                                                         MilliSecToSec(getRate()),
                                                         dq_ref, 0.5);
    }

    if(!control_computed) {
        ROS_ERROR("Error computing control");
    }
    return control_computed;
}

yarp::sig::Vector sot_VelKinCon_ctrl::getGravityCompensationTorque(const std::vector<string> &joint_names)
{
    yarp::sig::Vector tau(joint_names.size(), 0.0);
    int j = 0;
    for(unsigned int i = 0; i < joint_names.size(); ++i)
    {
        j = gravity_compensator_idynutils.coman_iDyn3.getDOFIndex(joint_names[i]);
        tau[i] = tau_gravity[j];
    }
    return tau;
}

yarp::sig::Vector sot_VelKinCon_ctrl::getGravityCompensationTorque(const yarp::sig::Vector q)
{
    static yarp::sig::Vector zeroes(q.size(),0.0);
    static yarp::sig::Vector tau(q.size(),0.0);

    gravity_compensator_idynutils.updateiDyn3Model(q,zeroes,zeroes);
    
    gravity_compensator_idynutils.coman_iDyn3.dynamicRNEA();
    tau = gravity_compensator_idynutils.coman_iDyn3.getTorques();

    return tau;
}

/**
 * Two-point formula: it computes the slope of a nearby secant line through the
 * points (x-h,f(x-h)) and (x+h,f(x+h))
 **/
yarp::sig::Vector sot_VelKinCon_ctrl::getGravityCompensationGradient(const yarp::sig::Matrix& W)
{
    //double start = yarp::os::Time::now();
    /// cost function is tau_g^t*tau_g
    static yarp::sig::Vector gradient(gravity_compensator_idynutils.coman_iDyn3.getNrOfDOFs(),0.0);
    static yarp::sig::Vector deltas(gravity_compensator_idynutils.coman_iDyn3.getNrOfDOFs(),0.0);
    for(unsigned int i = 0; i < gradient.size(); ++i)
    {
        // forward method gradient computation, milligrad
        const double h = 1E-3;
        deltas[i] = h;
        yarp::sig::Vector tau_gravity_q_a = getGravityCompensationTorque(q+deltas);
        yarp::sig::Vector tau_gravity_q_b = getGravityCompensationTorque(q-deltas);

        double C_g_q_a = yarp::math::dot(tau_gravity_q_a, W*tau_gravity_q_a);
        double C_g_q_b = yarp::math::dot(tau_gravity_q_b, W*tau_gravity_q_b);
        gradient[i] = (C_g_q_a - C_g_q_b)/(2*h);
        deltas[i] = 0;
    }

    //double elapsed = yarp::os::Time::now() - start;
    //ROS_WARN(" took %f ms", elapsed);
    return gradient;
}
