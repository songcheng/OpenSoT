#include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>
#include <yarp/math/Math.h>
#include <drc_shared/cartesian_utils.h>

using namespace yarp::math;

using namespace OpenSoT::constraints::velocity;

SelfCollisionAvoidance::SelfCollisionAvoidance(const yarp::sig::Vector &q, iDynUtils &robot):
    Constraint(q.size()),
    _robot(robot),
    _base_link("Waist"),
    _torso_frame("torso")
{

    computeKeypoints();

    update(q);

    printKeyPoints(_torso_key_points);
}

void SelfCollisionAvoidance::update(const yarp::sig::Vector& x)
{

}

bool SelfCollisionAvoidance::computeKeypoints()
{
    KDL::Frame I;
    _torso_key_points["DWYTorso"] = I;
    _torso_key_points[_torso_frame] = I;
    _torso_key_points["LShp"] = I;
    _torso_key_points["RShp"] = I;

    std::map<std::string, KDL::Frame>::iterator itr;
    int index = -1;
    KDL::Frame base_link_T_index_tmp;
    for (itr = _torso_key_points.begin(); itr != _torso_key_points.end(); itr++)
    {
        index = _robot.coman_iDyn3.getLinkIndex(itr->first);
        base_link_T_index_tmp = _robot.coman_iDyn3.getPositionKDL(_robot.coman_iDyn3.getLinkIndex(_base_link),
                                                                         index);

        itr->second = base_link_T_index_tmp;
    }
}

void SelfCollisionAvoidance::printKeyPoints(const std::map<std::string, KDL::Frame>& kp)
{
    std::map<std::string, KDL::Frame>::iterator itr;
    unsigned int i = 0;
    for (itr = _torso_key_points.begin(); itr != _torso_key_points.end(); itr++)
    {
        std::cout<<"key_point "<<i<<" "<<itr->first<<std::endl;
        cartesian_utils::printKDLFrame(itr->second);
        std::cout<<std::endl;

        i++;
    }
}

