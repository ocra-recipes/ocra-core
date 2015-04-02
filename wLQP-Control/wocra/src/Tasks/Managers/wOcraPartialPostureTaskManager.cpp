#include "wocra/Tasks/Managers/wOcraPartialPostureTaskManager.h"

namespace wocra
{

/** Base constructor
 *
 * \param ctrl                  wOcraController to connect to
 * \param model                 ocra model to setup the task
 * \param taskName              Name of the task
 * \param fullStateType         ocra::FullState enum specifying between (ocra::FullState::FULL_STATE, ocra::FullState::FREE_FLYER, ocra::FullState::INTERNAL)
 * \param dofIndices            Vector of indices for the DOFs to control
 * \param stiffness             Stiffness constant for task
 * \param damping               Damping constant for task
 * \param weight                Weight constant for task
 */
wOcraPartialPostureTaskManager::wOcraPartialPostureTaskManager(wOcraController& _ctrl, const wOcraModel& _model, const std::string& _taskName, int _fullStateType, Eigen::VectorXi& _dofIndices, double _stiffness, double _damping, double _weight) 
    : wOcraTaskManagerBase(_ctrl, _model, _taskName)
{
    _init(_fullStateType, _dofIndices, _stiffness, _damping, _weight);
}

/** Constructor with desired initial position
 *
 * \param ctrl                  wOcraController to connect to
 * \param model                 ocra model to setup the task
 * \param taskName              Name of the task
 * \param fullStateType         ocra::FullState enum specifying between (ocra::FullState::FULL_STATE, ocra::FullState::FREE_FLYER, ocra::FullState::INTERNAL)
 * \param dofIndices            Vector of indices for the DOFs to control
 * \param stiffness             Stiffness constant for task
 * \param damping               Damping constant for task
 * \param weight                Weight constant for task
 * \param init_q                Initial posture
 */
wOcraPartialPostureTaskManager::wOcraPartialPostureTaskManager(wOcraController& _ctrl, const wOcraModel& _model, const std::string& _taskName, int _fullStateType, Eigen::VectorXi& _dofIndices, double _stiffness, double _damping, double _weight, Eigen::VectorXd& _init_q) 
    : wOcraTaskManagerBase(_ctrl, _model, _taskName)
{
    _init(_fullStateType, _dofIndices, _stiffness, _damping, _weight);
    setPosture(_init_q);
}

/** Initializer function for constructor, sets up the frames, parameters, controller and task
 *
 */
void wOcraPartialPostureTaskManager::_init(int _fullStateType, VectorXi& _dofIndices, double _stiffness, double _damping, double _weight)
{
    featState = new wocra::PartialModelState(name + ".PartialModelState", model, _dofIndices, _fullStateType);
    featDesState = new wocra::PartialTargetState(name + ".PartialTargetState", model, _dofIndices, _fullStateType);
    feat = new wocra::PartialStateFeature(name + ".PartialStateFeature", *featState);
    featDes = new wocra::PartialStateFeature(name + ".PartialStateFeature_Des", *featDesState);

    // The feature initializes as Zero for posture
    task = &(ctrl.createwOcraTask(name, *feat, *featDes));
    task->initAsAccelerationTask();
    ctrl.addTask(*task);


    task->setStiffness(_stiffness);
    task->setDamping(_damping);
    task->setWeight(_weight);

    task->activateAsObjective();
}

/** Sets the partial joint space posture for the task
 *
 */
void wOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd& q)
{
    // Need to check size of q
    if (q.size() != featDesState->getSize())
    {
        throw std::runtime_error("[wOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd&)] Input pose and required dimension does not match");
    }
    featDesState->set_q(q);
    featDesState->set_qdot(Eigen::VectorXd::Zero(featDesState->getSize()));
    featDesState->set_qddot(Eigen::VectorXd::Zero(featDesState->getSize()));
}

/** Sets the partial joint space posture, velocity and acceleration for the task
 *
 */
void wOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd& q, Eigen::VectorXd& qdot, Eigen::VectorXd& qddot)
{
    // Need to check size of q
    if (q.size() != featDesState->getSize())
    {
        throw std::runtime_error("[wOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose and required dimension does not match");
    }
    else if (qdot.size() != featDesState->getSize())
    {
        throw std::runtime_error("[wOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose velocity and required dimension does not match");
    }
    else if (qddot.size() != featDesState->getSize())
    {
        throw std::runtime_error("[wOcraPartialPostureTaskManager::setPosture(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&)] Input pose acceleration and required dimension does not match");
    }

    featDesState->set_q(q);
    featDesState->set_qdot(qdot);
    featDesState->set_qddot(qddot);
}


/** Activate function
 *
 */
void wOcraPartialPostureTaskManager::activate()
{
    task->activateAsObjective();
}

/** Deactivate function
 *
 */
void wOcraPartialPostureTaskManager::deactivate()
{
    task->deactivate();
}

/** Sets the weight constant for this task
 *
 *  \param weight               Desired weight value
 */
void wOcraPartialPostureTaskManager::setWeight(double weight)
{
    task->setWeight(weight);
}

/** Gets the weight constant for this task
 *
 *  \return                     The weight for this task 
 */
double wOcraPartialPostureTaskManager::getWeight()
{
    Eigen::VectorXd weights = task->getWeight();
    return weights[0];
}

}
