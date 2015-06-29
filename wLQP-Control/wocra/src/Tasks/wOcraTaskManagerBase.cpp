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

        std::cout << "\n\n------\nOpening ports for "<< name <<":" << std::endl;

        port_in.open((portPrefix+":i").c_str());
        port_out.open((portPrefix+":o").c_str());

        std::cout << "------\n" << std::endl;
    }
    stateDimension = 0; // should be overwritten by derived classes who have implemented the necessary functions.
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
    // std::cout << "[wOcraTaskManagerBase::compileOutgoingMessage()]: compileOutgoingMessage has not been implemented or is not supported" << std::endl;
    // return false;

    // These should be in a separate header and possibly an enum or class or struct
    int BASE_MESSAGE_SIZE           = 12;
    int TM_MESSAGE_STIFFNESS        = 101;
    int TM_MESSAGE_DAMPING          = 102;
    int TM_MESSAGE_WEIGHT           = 103;
    int TM_MESSAGE_STATE_DIM        = 104;
    int TM_MESSAGE_STATE_CURRENT    = 105;
    int TM_MESSAGE_STATE_DESIRED    = 106;
    int TM_MESSAGE_IS_ACTIVATED     = 107;

    // stateDimension =
    messageLength = BASE_MESSAGE_SIZE + 2*stateDimension;



    yarp::os::Bottle& outputBottle = port_out.prepare();
    outputBottle.clear();
    outputBottle.addString("tmpName"); // should be name but this gives seg fault... Not sure why.
    outputBottle.addString(getTaskManagerType());
    outputBottle.addInt(messageLength);
    // Kp
    outputBottle.addInt(TM_MESSAGE_STIFFNESS);
    outputBottle.addDouble(getStiffness());

    // Kd
    outputBottle.addInt(TM_MESSAGE_DAMPING);
    outputBottle.addDouble(getDamping());

    // Weight
    outputBottle.addInt(TM_MESSAGE_WEIGHT);
    outputBottle.addDouble(getWeight());

    // State Dimension
    outputBottle.addInt(TM_MESSAGE_STATE_DIM);
    outputBottle.addInt(stateDimension);

    // // Current State

    updateCurrentStateVector(getCurrentState());

    outputBottle.addInt(TM_MESSAGE_STATE_CURRENT);
    for (int i=0; i < stateDimension; i++){
        outputBottle.addDouble(currentStateVector[i]);
    }

    // Desired State
    outputBottle.addInt(TM_MESSAGE_STATE_DESIRED);
    for (int i=0; i < stateDimension; i++){
        outputBottle.addDouble(desiredStateVector[i]);
    }

    // isActivated
    outputBottle.addInt(TM_MESSAGE_IS_ACTIVATED);
    outputBottle.addInt(checkIfActivated());


    return true;

}


std::string wOcraTaskManagerBase::getTaskManagerType()
{
    return "[wOcraTaskManagerBase::getTaskManagerType()]: getTaskManagerType has not been implemented for this task manager.";
}

void wOcraTaskManagerBase::setStateDimension(int taskDimension)
{
    stateDimension = taskDimension;
    currentStateVector.resize(stateDimension);
    desiredStateVector.resize(stateDimension);
    eigenCurrentStateVector = Eigen::VectorXd::Zero(stateDimension);
    eigenDesiredStateVector = Eigen::VectorXd::Zero(stateDimension);
}

void wOcraTaskManagerBase::updateCurrentStateVector(const double* ptrToFirstIndex)
{
    for(int i=0; i<stateDimension; i++){
        currentStateVector[i] = *ptrToFirstIndex;
        ptrToFirstIndex++;
    }
}

void wOcraTaskManagerBase::updateDesiredStateVector(const double* ptrToFirstIndex)
{
    for(int i=0; i<stateDimension; i++){
        desiredStateVector[i] = *ptrToFirstIndex;
        ptrToFirstIndex++;
    }
}

const double* wOcraTaskManagerBase::getCurrentState()
{
    Eigen::VectorXd emptyVector = Eigen::VectorXd::Zero(stateDimension);
    return emptyVector.data();
}


bool wOcraTaskManagerBase::checkIfActivated()
{
    return false;
}

}
