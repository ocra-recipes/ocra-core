#include "wocra/Tasks/Managers/wOcraSegPoseTaskManager.h"

namespace wocra
{

/** Base constructor
 *
 * \param _ctrl                  wOcraController to connect to
 * \param _model                 ocra model to setup the task
 * \param _taskName              Name of the task
 * \param _segmentName           Name of the segment for the task
 * \param _axes                  The axes used for the task
 * \param _stiffness             Stiffness constant for task
 * \param _damping               Damping constant for task
 * \param _weight                Weight constant for task
 */
wOcraSegPoseTaskManager::wOcraSegPoseTaskManager(wOcraController& _ctrl, const wOcraModel& _model, const std::string& _taskName, const std::string& _segmentName, ocra::ECartesianDof _axes, double _stiffness, double _damping, double _weight)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName), segmentName(_segmentName), axes(_axes)
{
    _init(Eigen::Displacementd::Identity(), _stiffness, _damping, _weight);
}

/** Constructor with desired pose
 *
 * \param _ctrl                 wOcraController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the task
 * \param _segmentName          Name of the segment for the task
 * \param _axes                 The axes used for the task
 * \param _stiffness            Stiffness constant for task
 * \param _damping              Damping constant for task
 * \param _weight               Weight constant for task
 * \param _poseDes              Initial pose for task
 */
wOcraSegPoseTaskManager::wOcraSegPoseTaskManager(wOcraController& _ctrl, const wOcraModel& _model, const std::string& _taskName, const std::string& _segmentName, ocra::ECartesianDof _axes, double _stiffness, double _damping, double _weight, const Eigen::Displacementd& _poseDes)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName), segmentName(_segmentName), axes(_axes)
{
    _init(Eigen::Displacementd::Identity(), _stiffness, _damping, _weight);
    setState(_poseDes);
}

/** Initializer function for the constructor, sets up the frames, parameters, controller and task
 *
 */
void wOcraSegPoseTaskManager::_init(const Eigen::Displacementd& _ref_LocalFrame, double _stiffness, double _damping, double _weight)
{
    featFrame = new ocra::SegmentFrame(name + ".SegmentFrame", model, model.SegmentName(segmentName), _ref_LocalFrame);
    featDesFrame = new ocra::TargetFrame(name + ".TargetFrame", model);
    feat = new ocra::DisplacementFeature(name + ".DisplacementFeature", *featFrame, axes);
    featDes = new ocra::DisplacementFeature(name + ".DisplacementFeature_Des", *featDesFrame, axes);

    task = &(ctrl.createwOcraTask(name, *feat, *featDes));
    task->initAsAccelerationTask();
    ctrl.addTask(*task);

    featDesFrame->setPosition(Eigen::Displacementd::Identity());
    featDesFrame->setVelocity(Eigen::Twistd::Zero());
    featDesFrame->setAcceleration(Eigen::Twistd::Zero());

    task->activateAsObjective();
    task->setStiffness(_stiffness);
    task->setDamping(_damping);
    task->setWeight(_weight);

    // Set the desired state to the current pose of the segment with 0 vel or acc
    setState(model.getSegmentPosition(model.getSegmentIndex(segmentName)));
}

/** Sets the pose for the task, both the translational and rotational components
 *
 * \param pose                  Vector for desired position
 */
void wOcraSegPoseTaskManager::setState(const Eigen::Displacementd& pose)
{
    setState(pose, Eigen::Twistd::Zero(), Eigen::Twistd::Zero());
}

/** Sets the pose, velocity and acceleration for the task, both the translational and rotational components
 *
 * \param pose                  Desired pose
 * \param velocity              Desired velocity
 * \param acceleration          Desired acceleration
 */
void wOcraSegPoseTaskManager::setState(const Eigen::Displacementd& pose, const Eigen::Twistd& velocity, const Eigen::Twistd& acceleration)
{
    featDesFrame->setPosition(pose);
    featDesFrame->setVelocity(velocity);
    featDesFrame->setAcceleration(acceleration);
}

/** Sets the weight constant for this task
 *
 *  \param weight               Desired weight value
 */
void wOcraSegPoseTaskManager::setWeight(double weight)
{
    task->setWeight(weight);
}

/** Gets the weight constant for this task
 *
 *  \return                     The weight for this task 
 */
double wOcraSegPoseTaskManager::getWeight()
{
    Eigen::VectorXd weights = task->getWeight();
    return weights[0];
}

/** Sets the stiffness for this task
 *
 * \param stiffness             Desired stiffness
 */
void wOcraSegPoseTaskManager::setStiffness(double stiffness)
{
    task->setStiffness(stiffness);
}

/** Gets the stiffness constant for this task
 *
 *  \return                     The stiffness for this task
 */
double wOcraSegPoseTaskManager::getStiffness()
{
    Eigen::MatrixXd K = task->getStiffness();
    return K(0, 0);
}

/** Sets the damping for this task
 *
 * \param damping               Desired damping
 */
void wOcraSegPoseTaskManager::setDamping(double damping)
{
    task->setDamping(damping);
}

/** Gets the damping constant for this task
 *
 *  \return                     The damping for this task
 */
double wOcraSegPoseTaskManager::getDamping()
{
    Eigen::MatrixXd C = task->getDamping();
    return C(0, 0);
}

/** Gets the error for this task (COM position error)
 *
 */
Eigen::VectorXd wOcraSegPoseTaskManager::getTaskError()
{
    return task->getError();
}


/** Activates the task
 *
 */
void wOcraSegPoseTaskManager::activate()
{
    task->activateAsObjective();
}

/** Deactivates the task
 *
 */
void wOcraSegPoseTaskManager::deactivate()
{
    task->deactivate();
}

}
