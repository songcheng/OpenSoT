#ifndef __BOUNDS_VELOCITY_SELFCOLLISIONAVOIDANCE_H__
#define __BOUNDS_VELOCITY_SELFCOLLISIONAVOIDANCE_H__

 #include <OpenSoT/Constraint.h>

 #include <yarp/sig/all.h>
 #include <iCub/iDynTree/DynTree.h>

 namespace OpenSoT {
    namespace constraints {
        namespace velocity {
            /**
             * @brief The SelfCollisionAvoidance class implements an algorithm to prevent links collision
             */
            class SelfCollisionAvoidance: public Constraint<yarp::sig::Matrix, yarp::sig::Vector> {
            private:

            public:
                SelfCollisionAvoidance(const yarp::sig::Vector &q,
                            const double boundScaling = 1.0);

                void update(const yarp::sig::Vector &x);

            };
        }
    }
 }

#endif
