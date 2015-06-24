#include "wocra/Tasks/wOcraTaskManagerBase.h"

namespace wocra
{

/** base constructor
 *
 * \param ctrl                  wOcraController to connect to
 * \param model                 ocra model to setup the task
 * \param taskName              Name of the task
 */
wOcraTaskManagerBase::wOcraTaskManagerBase(wOcraController& _ctrl, const wOcraModel& _model, const std::string& _taskName)//, bool _usesYarpPorts)
    : ctrl(_ctrl), model(_model), name(_taskName)//, usesYARP(_usesYarpPorts)
{
    usesYARP =true;
    if (usesYARP) {
        std::string portPrefix = "/TM/"+name;

        std::cout << "\n------\nOpening ports for "<< name <<":" << std::endl;

        port_in.open((portPrefix+":i").c_str());
        port_out.open((portPrefix+":o").c_str());

        std::cout << "------" << std::endl;
    }
}

/** Returns the error vector of the task
 *
 *  If the derived child class does not have a meaningful error, it should override this function to throw an error
 */
Eigen::VectorXd wOcraTaskManagerBase::getTaskError()
{
    throw std::runtime_error(std::string("[wOcraTaskManagerBase::getTaskError()]: getTaskError has not been implemented or is not supported"));
}

/** Returns the norm of the error vector
 *
 *  If the derived child class does not have a meaningful error, it should override this function to throw an error
 */
double wOcraTaskManagerBase::getTaskErrorNorm()
{
    return getTaskError().norm();
}

void wOcraTaskManagerBase::refreshPorts()
{
    if (usesYARP) {
        yarp::os::Bottle *input = port_in.read(false);
        if (input != NULL) {
            parseIncomingMessage(input);
        }

        if(compileOutgoingMessage())
        {
            port_out.write();
        }

    }
}

void wOcraTaskManagerBase::parseIncomingMessage(yarp::os::Bottle *input)
{
    std::cout << "[wOcraTaskManagerBase::parseIncomingMessage()]: parseIncomingMessage has not been implemented or is not supported for this task manager." << std::endl;
}


bool wOcraTaskManagerBase::compileOutgoingMessage()
{
    std::cout << "[wOcraTaskManagerBase::compileOutgoingMessage()]: compileOutgoingMessage has not been implemented or is not supported" << std::endl;
    return false;
}


std::string wOcraTaskManagerBase::getTaskManagerType()
{
    return "[wOcraTaskManagerBase::getTaskManagerType()]: getTaskManagerType has not been implemented for this task manager.";
}





}
