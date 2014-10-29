#ifndef __BOUNDS_VELOCITY_SELFCOLLISIONAVOIDANCE_H__
#define __BOUNDS_VELOCITY_SELFCOLLISIONAVOIDANCE_H__

 #include <OpenSoT/Constraint.h>

 #include <yarp/sig/all.h>
 #include <iCub/iDynTree/DynTree.h>
 #include <drc_shared/idynutils.h>

 namespace OpenSoT {
    namespace constraints {
        namespace velocity {
            /**
             * @brief The SelfCollisionAvoidance class implements an algorithm to prevent links collision
             */
            class SelfCollisionAvoidance: public Constraint<yarp::sig::Matrix, yarp::sig::Vector> {
            private:
                iDynUtils& _robot;

                std::string _base_link;
                std::string _torso_frame;

                std::map<std::string, KDL::Frame> _torso_key_points;


                bool computeKeypoints();
            public:
                SelfCollisionAvoidance(const yarp::sig::Vector &q, iDynUtils& robot);

                void update(const yarp::sig::Vector &x);

                void printKeyPoints(const std::map<std::string, KDL::Frame>& kp);


            };
        }
    }
 }

#endif
