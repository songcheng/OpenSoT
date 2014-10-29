#include "OpenSoT/constraints/velocity/SelfCollisionAvoidance.h"
#include <yarp/math/Math.h>

using namespace yarp::math;

using namespace OpenSoT::constraints::velocity;

SelfCollisionAvoidance::SelfCollisionAvoidance(const yarp::sig::Vector &q, const double boundScaling):
    Constraint(q.size())
{
    update(q);
}

void SelfCollisionAvoidance::update(const yarp::sig::Vector& x)
{


}

