#ifndef wOcraTASKMANAGERBASE_H
#define wOcraTASKMANAGERBASE_H

#include "wocra/Models/wOcraModel.h"
#include "wocra/wOcraController.h"
#include "wocra/Tasks/wOcraTask.h"

#include <Eigen/Dense>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>

namespace wocra
{

/** \brief Task Manager for the Center of Mass (CoM) task with the wOcra Controller
 *
 */
class wOcraTaskManagerBase
{
    public:
        wOcraTaskManagerBase(wocra::wOcraController& ctrl, const wocra::wOcraModel& model, const std::string& name, bool usesYarpPorts=false);
        ~wOcraTaskManagerBase();


        virtual void activate() = 0;
        virtual void deactivate() = 0;




        // For getting the task type
        virtual std::string getTaskManagerType();

        virtual VectorXd getTaskError();
        double getTaskErrorNorm();

        void refreshPorts();


    protected:
        wocra::wOcraController&        ctrl;
        const wocra::wOcraModel&       model;
        const std::string&              name;
        std::string                     stableName; //hack to avoid using name in compileOutgoingMessage()

        //Generic double vector to store states:
        bool taskManagerActive;

        std::vector<double> currentStateVector, desiredStateVector;

        Eigen::VectorXd eigenCurrentStateVector, eigenDesiredStateVector;

        virtual void setStiffness(double stiffness){ std::cout << "setStiffness() Not implemented" << std::endl; }
        virtual double getStiffness(){return 0.0;}
        virtual void setDamping(double damping){ std::cout << "setDamping() Not implemented" << std::endl; }
        virtual double getDamping(){return 0.0;}
        virtual void setWeight(double weight){ std::cout << "setWeight() Not implemented" << std::endl; }
        virtual double getWeight(){return 0.0;}

        virtual void setWeights(Eigen::Vector3d weight){};
        virtual Eigen::VectorXd getWeights(){};


        virtual const double* getCurrentState();
        virtual bool checkIfActivated();

        void updateDesiredStateVector(const double* ptrToFirstIndex);
        void updateCurrentStateVector(const double* ptrToFirstIndex);

        void setStateDimension(int taskDimension);


        // For parsing and compiling yarp messages.
        virtual void parseIncomingMessage(yarp::os::Bottle *input);
        virtual bool compileOutgoingMessage();

        void printValidMessageTags();


        bool usesYARP;
        yarp::os::Network yarp;
        yarp::os::BufferedPort<yarp::os::Bottle> port_in;
        yarp::os::BufferedPort<yarp::os::Bottle> port_out;
        int messageLength, stateDimension;


};

}

#endif // wOcraTASKMANAGERBASE_H
