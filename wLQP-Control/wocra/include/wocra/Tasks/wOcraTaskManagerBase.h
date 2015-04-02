#ifndef wOcraTASKMANAGERBASE_H
#define wOcraTASKMANAGERBASE_H

#include "wocra/Models/wOcraModel.h"
#include "wocra/wOcraController.h"

#include <Eigen/Dense>

namespace wocra
{

/** \brief Task Manager for the Center of Mass (CoM) task with the wOcra Controller
 *
 */
class wOcraTaskManagerBase
{
    public:
        wOcraTaskManagerBase(wocra::wOcraController& ctrl, const wocra::wOcraModel& model, const std::string& name);

        virtual void activate() = 0;
        virtual void deactivate() = 0;

        virtual VectorXd getTaskError();
        double getTaskErrorNorm();
 
    protected:
        wocra::wOcraController&        ctrl;
        const wocra::wOcraModel&       model;
        const std::string&              name;
        //wocra::wOcraTask              task;
};

}

#endif // wOcraTASKMANAGERBASE_H
