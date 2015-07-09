#include "wocra/Tasks/wOcraTaskManagerBase.h"

#define BASE_MESSAGE_SIZE 12

namespace wocra
{

/** base constructor
 *
 * \param ctrl                  wOcraController to connect to
 * \param model                 ocra model to setup the task
 * \param taskName              Name of the task
 */
wOcraTaskManagerBase::wOcraTaskManagerBase(wOcraController& _ctrl, const wOcraModel& _model, const std::string& _taskName, bool _usesYarpPorts)
    : ctrl(_ctrl), model(_model), name(_taskName), usesYARP(_usesYarpPorts)
{
    stableName = name;
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


wOcraTaskManagerBase::~wOcraTaskManagerBase()
{
    port_in.close();
    port_out.close();
    std::cout << stableName<<" destroyed" << std::endl;
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
            input->clear();
        }
    }
}

void wOcraTaskManagerBase::parseIncomingMessage(yarp::os::Bottle *input)
{
    int btlSize = input->size();

    for (int i=0; i<btlSize;)
    {
        std::string msgTag = input->get(i).asString();

        if(msgTag == "publish")
        {
            if(compileOutgoingMessage())
            {
                port_out.write();
            }

            i++;
        }
        else if(msgTag == "stiffness")
        {
            i++;
            setStiffness(input->get(i).asDouble());
            i++;
        }
        else if (msgTag == "damping")
        {
            i++;
            setDamping(input->get(i).asDouble());
            i++;
        }
        else if (msgTag == "weight")
        {
            i++;
            setWeight(input->get(i).asDouble());
            i++;
        }
        else if (msgTag == "desired_state")
        {
            std::cout << "Implement setDesiredState..." << std::endl;
        }
        else if (msgTag == "activate")
        {
            activate();
        }
        else if (msgTag == "deactivate")
        {
            deactivate();
        }
        else if (msgTag == "state_dimension")
        {
            std::cout << "The task, " << stableName << " has state_dimension = " << stateDimension << "." << std::endl;
        }
        else if (msgTag == "help")
        {
            printValidMessageTags();
        }
        else
        {
            std::cout << "[ERROR] (wOcraTaskManagerBase::parseIncomingMessage): Aborting. The message tag, " << msgTag << " doesn't exist. Here is some help..." << std::endl;
            // printValidMessageTags();
            // i=btlSize;
        }
    }
}

void wOcraTaskManagerBase::printValidMessageTags()
{
    std::cout << "\n=== Valid message tags are: ===" << std::endl;
    std::cout << "stiffness  <-- Allows you to set the Kp gain. Expects 1 double value" << std::endl;
    std::cout << "damping  <-- Allows you to set the Kd gain. Expects 1 double value" << std::endl;
    std::cout << "weight  <-- Allows you to set the task weight. Expects 1 double value" << std::endl;
    std::cout << "desired_state  <-- Allows you to set the desired task reference. Expects nDoF double values where nDoF is the task dimension" << std::endl;
    std::cout << "activate  <-- Allows you to activate the task. No arguments expected" << std::endl;
    std::cout << "deactivate  <-- Allows you to deactivate the task. No arguments expected" << std::endl;
    std::cout << "state_dimension  <-- Prints the state dimension. No arguments expected" << std::endl;
    std::cout << "help  <-- Prints all the valid commands. No arguments expected" << std::endl;

    std::cout << "\nTypical usage: [message tag] [message value(s)]\ne.g.\t >> stiffness 20 damping 10 desired_state 1.0 2.0 2.0" << std::endl;
    std::cout << "\n\t FYI: your state dimension is: " << stateDimension << std::endl;
}

bool wOcraTaskManagerBase::compileOutgoingMessage()
{

    messageLength = BASE_MESSAGE_SIZE + 2*stateDimension;



    yarp::os::Bottle& outputBottle = port_out.prepare();
    outputBottle.clear();
    //TODO: using "name" gives seg fault... Not sure why. this is a hack.
    outputBottle.addString(stableName);
    outputBottle.addString(getTaskManagerType());
    outputBottle.addInt(messageLength);
    // Kp
    outputBottle.addString("stiffness");
    outputBottle.addDouble(getStiffness());

    // Kd
    outputBottle.addString("damping");
    outputBottle.addDouble(getDamping());

    // Weight
    outputBottle.addString("weight");
    outputBottle.addDouble(getWeight());

    // State Dimension
    outputBottle.addString("state_dimension");
    outputBottle.addInt(stateDimension);

    // // Current State

    updateCurrentStateVector(getCurrentState());

    outputBottle.addString("current_state");
    for (int i=0; i < stateDimension; i++){
        outputBottle.addDouble(currentStateVector[i]);
    }

    // Desired State
    outputBottle.addString("desired_state");
    for (int i=0; i < stateDimension; i++){
        outputBottle.addDouble(desiredStateVector[i]);
    }

    // isActivated
    outputBottle.addString("activation_status");
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
