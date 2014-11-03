#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/interfaces/yarp/tasks/YCartesian.h>
#include <drc_shared/tests_utils.h>
#include <yarp/math/Math.h>

using namespace yarp::math;

namespace {

class testYTask : public ::testing::Test{
 protected:

  testYTask():
      q(robot.coman_iDyn3.getNrOfDOFs(), 0.0)
  {
      robot.updateiDyn3Model(q, true);
  }

  virtual ~testYTask()
  {}

  virtual void SetUp()
  {}

  virtual void TearDown()
  {}

  iDynUtils robot;
  yarp::sig::Vector q;
};

TEST_F(testYTask, testYTASK)
{
    std::string robot_name = "coman";
    std::string module_prefix = "sot_velkincon";
    std::string task_id = "cartesian::left_arm";
    std::string base_link = "Waist";
    std::string distal_link = "l_wrist";

    if(tests_utils::startYarpServer())
    {
        ::yarp::os::Network yarp_network;

        unsigned int number_of_trials = 1000;
        bool check = false;
        for(unsigned int i = 0; i < number_of_trials; ++i)
        {
            if(yarp_network.checkNetwork())
            {
                check = true;
                break;
            }
        }
        if(check)
        {
            OpenSoT::interfaces::yarp::tasks::YCartesian y_cartesian_l_arm(robot_name, module_prefix, task_id, q, robot, distal_link, base_link);

            EXPECT_EQ(y_cartesian_l_arm.getBaseLink(), base_link);
            EXPECT_EQ(y_cartesian_l_arm.getDistalLink(), distal_link);
            EXPECT_EQ(y_cartesian_l_arm.getTaskID(), task_id);
            std::string port_prefix("/"+robot_name+"/"+module_prefix+"/"+task_id+"/");
            EXPECT_EQ(y_cartesian_l_arm.getPortPrefix(), port_prefix);

            EXPECT_TRUE(yarp_network.exists(port_prefix+"set_ref:i"));
            EXPECT_TRUE(yarp_network.exists(port_prefix+"rpc"));

            double lambda = 0.5;
            yarp::os::RpcClient rpc;
            rpc.open("/rpc_test");
            EXPECT_TRUE(yarp_network.connect("/rpc_test", port_prefix+"rpc"));
            yarp::os::Bottle b_in, b_out;
            b_out.addString("set lambda");
            b_out.addDouble(lambda);
            sleep(1);
            rpc.write(b_out, b_in);
            EXPECT_TRUE(b_in.get(0).asInt() == OpenSoT::interfaces::yarp::tasks::RPCCallBack::SUCCEED);
            b_out.clear();b_in.clear();
            b_out.addString("get lambda");
            rpc.write(b_out, b_in);
            sleep(1);
            EXPECT_DOUBLE_EQ(b_in.get(0).asDouble(), lambda);
            b_out.clear();b_in.clear();
            b_out.addString("set lambda");
            b_out.addDouble(2.0);
            rpc.write(b_out, b_in);
            sleep(1);
            EXPECT_TRUE(b_in.get(0).asInt() == OpenSoT::interfaces::yarp::tasks::RPCCallBack::ERROR_LAMBA_GAIN_MORE_THAN_1);
            b_out.clear();b_in.clear();
            b_out.addString("get lambda");
            rpc.write(b_out, b_in);
            sleep(1);
            EXPECT_DOUBLE_EQ(b_in.get(0).asDouble(), lambda);
            b_out.clear();b_in.clear();
            b_out.addString("set lambda");
            b_out.addDouble(-0.5);
            rpc.write(b_out, b_in);
            sleep(1);
            EXPECT_TRUE(b_in.get(0).asInt() == OpenSoT::interfaces::yarp::tasks::RPCCallBack::ERROR_NEGATIVE_LAMBDA_GAIN);

            double orientation_gain = 10.5;
            yarp::os::Bottle b_in2, b_out2;
            b_out2.addString("set orientation_gain");
            b_out2.addDouble(orientation_gain);
            sleep(1);
            rpc.write(b_out2, b_in2);
            EXPECT_TRUE(b_in2.get(0).asInt() == OpenSoT::interfaces::yarp::tasks::RPCCallBack::SUCCEED);
            b_out2.clear();b_in2.clear();
            b_out2.addString("get orientation_gain");
            rpc.write(b_out2, b_in2);
            sleep(1);
            EXPECT_DOUBLE_EQ(b_in2.get(0).asDouble(), orientation_gain);
            b_out2.clear();b_in2.clear();
            b_out2.addString("set orientation_gain");
            b_out2.addDouble(-2.0);
            rpc.write(b_out2, b_in2);
            sleep(1);
            EXPECT_TRUE(b_in2.get(0).asInt() == OpenSoT::interfaces::yarp::tasks::RPCCallBack::ERROR_NEGATIVE_ORIENTATION_GAIN);
            b_out2.clear();b_in2.clear();
            b_out2.addString("get orientation_gain");
            rpc.write(b_out2, b_in2);
            sleep(1);
            EXPECT_DOUBLE_EQ(b_in2.get(0).asDouble(), orientation_gain);

            yarp::sig::Matrix W(2, 2);
            W = 2.0*W.eye();
            yarp::os::Bottle b_in3, b_out3;
            b_out3.addString("set W");
            for(unsigned int i = 0; i < W.rows(); ++i)
                b_out3.addDouble(W(i,i));
            rpc.write(b_out3, b_in3);
            sleep(1);
            EXPECT_TRUE(b_in3.get(0).asInt() == OpenSoT::interfaces::yarp::tasks::RPCCallBack::ERROR_WRONG_VECTOR_SIZE);
            b_out3.clear();b_in3.clear();
            b_out3.addString("get W");
            rpc.write(b_out3, b_in3);
            sleep(1);
            W.resize(b_in3.size(), b_in3.size());
            W.zero();
            for(unsigned int i = 0; i < b_in3.size(); ++i)
                W(i,i) = b_in3.get(i).asDouble();
            EXPECT_TRUE(W == y_cartesian_l_arm.taskCartesian->getWeight());
            b_out3.clear();b_in3.clear();
            b_out3.addString("set W");
            W = -1.0*W;
            for(unsigned int i = 0; i < W.rows(); ++i)
                b_out3.addDouble(W(i,i));
            rpc.write(b_out3, b_in3);
            sleep(1);
            EXPECT_TRUE(b_in3.get(0).asInt() == OpenSoT::interfaces::yarp::tasks::RPCCallBack::ERROR_NEGATIVE_W_GAIN);
            b_out3.clear();b_in3.clear();
            b_out3.addString("set W");
            W = -2.0*W;
            for(unsigned int i = 0; i < W.rows(); ++i)
                b_out3.addDouble(W(i,i));
            rpc.write(b_out3, b_in3);
            sleep(1);
            EXPECT_TRUE(b_in3.get(0).asInt() == OpenSoT::interfaces::yarp::tasks::RPCCallBack::SUCCEED);
            b_out3.clear();b_in3.clear();
            b_out3.addString("get W");
            rpc.write(b_out3, b_in3);
            sleep(1);
            W.resize(b_in3.size(), b_in3.size());
            W.zero();
            for(unsigned int i = 0; i < b_in3.size(); ++i)
                W(i,i) = b_in3.get(i).asDouble();
            EXPECT_TRUE(W == y_cartesian_l_arm.taskCartesian->getWeight());

            y_cartesian_l_arm.cleanPorts();

            robot_name = "";
            OpenSoT::tasks::velocity::Cartesian::Ptr cartesian_l_arm(
                        new OpenSoT::tasks::velocity::Cartesian(task_id, q, robot, distal_link, base_link));
            OpenSoT::interfaces::yarp::tasks::YCartesian y_cartesian_l_arm2(robot_name, module_prefix, cartesian_l_arm);
            EXPECT_EQ(y_cartesian_l_arm2.getBaseLink(), base_link);
            EXPECT_EQ(y_cartesian_l_arm2.getDistalLink(), distal_link);
            EXPECT_EQ(y_cartesian_l_arm2.getTaskID(), task_id);
            port_prefix = "/"+module_prefix+"/"+task_id+"/";
            EXPECT_EQ(y_cartesian_l_arm2.getPortPrefix(), port_prefix);
            y_cartesian_l_arm2.cleanPorts();

            robot_name = "";
            module_prefix = "";
            task_id = "cartesian::left_arm";
            OpenSoT::interfaces::yarp::tasks::YCartesian y_cartesian_l_arm4(robot_name, module_prefix, task_id, q, robot, distal_link, base_link);
            EXPECT_EQ(y_cartesian_l_arm4.getBaseLink(), base_link);
            EXPECT_EQ(y_cartesian_l_arm4.getDistalLink(), distal_link);
            EXPECT_EQ(y_cartesian_l_arm4.getTaskID(), task_id);
            port_prefix = "/"+task_id+"/";
            EXPECT_EQ(y_cartesian_l_arm4.getPortPrefix(), port_prefix);
            y_cartesian_l_arm4.cleanPorts();

            std::cout<<"INIT FRAME: "<<std::endl;cartesian_utils::printHomogeneousTransform(y_cartesian_l_arm4.taskCartesian->getReference());
            std::cout<<"base_frame: "<<y_cartesian_l_arm4.taskCartesian->getBaseLink()<<std::endl;
            std::cout<<"distal_frame: "<<y_cartesian_l_arm4.taskCartesian->getDistalLink()<<std::endl;std::cout<<std::endl;

            yarp::os::BufferedPort<OpenSoT::interfaces::yarp::msgs::yarp_trj_msg_portable> trj_msg_port;
            trj_msg_port.open("/test_port:o");
            sleep(1);
            yarp_network.connect("/test_port:o", y_cartesian_l_arm4.getPortPrefix()+"set_ref:i");
            sleep(1);
            OpenSoT::interfaces::yarp::msgs::yarp_trj_msg_portable& trj_msg = trj_msg_port.prepare();
            trj_msg.base_frame = base_link;
            trj_msg.distal_frame = distal_link;
            trj_msg.pose.p[0] = 1.0;
            trj_msg.pose.p[1] = 2.0;
            trj_msg.pose.p[2] = 3.0;
            trj_msg.pose.M.DoRotX(M_PI);
            trj_msg_port.write();
            sleep(1);

            std::cout<<"SENT FRAME: "<<std::endl;cartesian_utils::printKDLFrame(trj_msg.pose);
            std::cout<<"base_frame: "<<trj_msg.base_frame<<std::endl;
            std::cout<<"distal_frame: "<<trj_msg.distal_frame<<std::endl;std::cout<<std::endl;

            std::cout<<"RECEIVED FRAME: "<<std::endl;cartesian_utils::printHomogeneousTransform(y_cartesian_l_arm4.taskCartesian->getReference());
            std::cout<<"base_frame: "<<y_cartesian_l_arm4.taskCartesian->getBaseLink()<<std::endl;
            std::cout<<"distal_frame: "<<y_cartesian_l_arm4.taskCartesian->getDistalLink()<<std::endl;std::cout<<std::endl;

            KDL::Frame tmp;
            cartesian_utils::fromYARPMatrixtoKDLFrame(y_cartesian_l_arm4.taskCartesian->getReference(), tmp);
            EXPECT_TRUE(tmp == trj_msg.pose);
            EXPECT_TRUE(y_cartesian_l_arm4.taskCartesian->getBaseLink() == trj_msg.base_frame);
            EXPECT_TRUE(y_cartesian_l_arm4.taskCartesian->getDistalLink() == trj_msg.distal_frame);


        }
        tests_utils::stopYarpServer();

    }

}

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
