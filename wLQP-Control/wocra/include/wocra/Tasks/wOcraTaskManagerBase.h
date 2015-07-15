#ifndef wOcraTASKMANAGERBASE_H
#define wOcraTASKMANAGERBASE_H

#include "wocra/Models/wOcraModel.h"
#include "wocra/wOcraController.h"
#include "wocra/Tasks/wOcraTask.h"

#include <Eigen/Dense>

#include <yarp/os/Network.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/ConnectionReader.h>

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

        std::string getPortName();



        // For getting the task type
        virtual std::string getTaskManagerType();

        virtual VectorXd getTaskError();
        double getTaskErrorNorm();


        /************** DataProcessor *************/
        class DataProcessor : public yarp::os::PortReader {
            private:
                wOcraTaskManagerBase& tmBase;

            public:
                DataProcessor(wOcraTaskManagerBase& tmBaseRef);

                virtual bool read(yarp::os::ConnectionReader& connection);
        };
        /************** DataProcessor *************/


    protected:
        wocra::wOcraTask*              task;


        wocra::wOcraController&        ctrl;
        const wocra::wOcraModel&       model;
        const std::string&              name;
        std::string                     stableName; //hack to avoid using name in compileOutgoingMessage()

        //Generic double vector to store states:
        bool taskManagerActive;

        std::vector<double> currentStateVector, desiredStateVector, newDesiredStateVector;

        Eigen::VectorXd eigenCurrentStateVector, eigenDesiredStateVector;

        virtual void setStiffness(double stiffness){ std::cout << "setStiffness() Not implemented" << std::endl; }
        virtual double getStiffness(){return 0.0;}
        virtual void setDamping(double damping){ std::cout << "setDamping() Not implemented" << std::endl; }
        virtual double getDamping(){return 0.0;}
        virtual void setWeight(double weight){ std::cout << "setWeight() Not implemented" << std::endl; }
        virtual double getWeight(){return 0.0;}
        virtual void setDesiredState(){ std::cout << "setDesiredState() Not implemented" << std::endl; }

        virtual void setWeights(Eigen::Vector3d weight){};
        virtual Eigen::VectorXd getWeights(){};


        virtual const double* getCurrentState();
        virtual bool checkIfActivated();

        void updateDesiredStateVector(const double* ptrToFirstIndex);
        void updateCurrentStateVector(const double* ptrToFirstIndex);

        void setStateDimension(int taskDimension);


        // For parsing and compiling yarp messages.
        virtual void parseIncomingMessage(yarp::os::Bottle *input, yarp::os::Bottle *reply);

        std::string printValidMessageTags();


        bool usesYARP;
        yarp::os::Network yarp;
        yarp::os::RpcServer rpcPort;
        std::string portName;
        DataProcessor processor;

        int stateDimension;



};

}

#endif // wOcraTASKMANAGERBASE_H
