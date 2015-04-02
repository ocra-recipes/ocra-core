#include "wocra/Tasks/wOcraTaskManagerBase.h"

namespace wocra
{

/** base constructor
 *
 * \param ctrl                  wOcraController to connect to
 * \param model                 ocra model to setup the task
 * \param taskName              Name of the task
 */
wOcraTaskManagerBase::wOcraTaskManagerBase(wOcraController& _ctrl, const wOcraModel& _model, const std::string& _taskName) 
    : ctrl(_ctrl), model(_model), name(_taskName)
{
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

}
