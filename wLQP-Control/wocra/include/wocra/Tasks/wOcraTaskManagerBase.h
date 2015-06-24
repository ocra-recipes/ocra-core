#ifndef wOcraTASKMANAGERBASE_H
#define wOcraTASKMANAGERBASE_H

#include "wocra/Models/wOcraModel.h"
#include "wocra/wOcraController.h"

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
        wOcraTaskManagerBase(wocra::wOcraController& ctrl, const wocra::wOcraModel& model, const std::string& name);//, bool usesYarpPorts=false);

        virtual void activate() = 0;
        virtual void deactivate() = 0;

        // For parsing and compiling yarp messages.
        virtual void parseIncomingMessage(yarp::os::Bottle *input);
        virtual bool compileOutgoingMessage();


        // For getting the task type
        virtual std::string getTaskManagerType();

        virtual VectorXd getTaskError();
        double getTaskErrorNorm();

        void refreshPorts();


    protected:
        wocra::wOcraController&        ctrl;
        const wocra::wOcraModel&       model;
        const std::string&              name;
        //wocra::wOcraTask              task;



        bool usesYARP;
        yarp::os::Network yarp;
        yarp::os::BufferedPort<yarp::os::Bottle> port_in;
        yarp::os::BufferedPort<yarp::os::Bottle> port_out;



};

}

#endif // wOcraTASKMANAGERBASE_H
