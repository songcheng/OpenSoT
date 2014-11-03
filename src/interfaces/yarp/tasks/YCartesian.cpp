#include <OpenSoT/interfaces/yarp/tasks/YCartesian.h>

using namespace OpenSoT::interfaces::yarp::tasks;

YCartesian::YCartesian(const std::string &robot_name, const std::string &module_prefix, std::string task_id,
                       const ::yarp::sig::Vector &x, iDynUtils &robot, std::string distal_link, std::string base_link):
    ::yarp::os::BufferedPort<msgs::yarp_trj_msg_portable>(),
    taskCartesian(new Cartesian(task_id,x,robot, distal_link, base_link)),
    _port_prefix(),
    _rpc_cb(taskCartesian),
    _rpc()
{
    assert(computePortPrefix(robot_name, module_prefix, task_id));
    open(_port_prefix+"set_ref:i");

    useCallback();
    _rpc.setReader(_rpc_cb);
    _rpc.open(_port_prefix+"rpc");
}

YCartesian::YCartesian(const std::string& robot_name,
                       const std::string& module_prefix,
                       Cartesian::Ptr cartesian_task):
    ::yarp::os::BufferedPort<msgs::yarp_trj_msg_portable>(),
    taskCartesian(cartesian_task),
    _port_prefix(),
    _rpc_cb(taskCartesian),
    _rpc()
{
    assert(computePortPrefix(robot_name, module_prefix, cartesian_task->getTaskID()));
    open(_port_prefix+"set_ref:i");
    useCallback();

    _rpc.setReader(_rpc_cb);
    _rpc.open(_port_prefix+"rpc");
}

void YCartesian::cleanPorts()
{
    std::cout<<"Cleaning Ports for "<<taskCartesian->getTaskID()<<"...";

    ::yarp::os::Bottle* foo;

    int pendings = getPendingReads();
    for(unsigned int i = 0; i < pendings; ++i)
        read(foo);

    std::cout<<"clean!"<<std::endl;
}

bool YCartesian::computePortPrefix(const std::string& robot_name, const std::string& module_prefix, const std::string &task_id)
{
    _port_prefix = "/";
    if(robot_name != "")
        _port_prefix += robot_name+"/";
    if(module_prefix != "")
        _port_prefix += module_prefix+"/";
    if(task_id == "")
    {
        std::cout<<"ERROR: task_id can not be an empty string!"<<std::endl;
        return false;
    }
    _port_prefix += task_id+"/";
    return true;
}

void YCartesian::onRead(msgs::yarp_trj_msg_portable& ref_trj_msg)
{
    if(!(ref_trj_msg.base_frame == taskCartesian->getBaseLink()))
        std::cout<<"WARNING: Reference Trajectory has "<<ref_trj_msg.base_frame<<
                   " instead of "<<taskCartesian->getBaseLink()<<" as base_frame"<<std::endl;
    else
    {
        if(!(ref_trj_msg.distal_frame == taskCartesian->getDistalLink()))
            std::cout<<"WARNING: Reference Trajectory has "<<ref_trj_msg.distal_frame<<
                       " instead of "<<taskCartesian->getDistalLink()<<" as distal_frame"<<std::endl;
        else
        {
            ::yarp::sig::Matrix tmp;
            cartesian_utils::fromKDLFrameToYARPMatrix(ref_trj_msg.pose, tmp);
            taskCartesian->setReference(tmp);
        }
    }
}
