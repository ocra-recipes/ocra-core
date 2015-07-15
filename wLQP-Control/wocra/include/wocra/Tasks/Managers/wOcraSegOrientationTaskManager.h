#ifndef wOcraSEGORIENTATIONTASKMANAGER_H
#define wOcraSEGORIENTATIONTASKMANAGER_H

#include "ocra/control/Model.h"
#include "wocra/Tasks/wOcraTaskManagerBase.h"
#include "wocra/Tasks/wOcraTask.h"
#include "wocra/wOcraController.h"

#include <Eigen/Dense>

namespace wocra
{

/** \brief Task Manager for the Center of Mass (CoM) task with the wOcra Controller
 *
 */
class wOcraSegOrientationTaskManager: public wOcraTaskManagerBase
{
    public:
        wOcraSegOrientationTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, double stiffness, double damping, double weight, bool usesYarpPorts = false);

        wOcraSegOrientationTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, double stiffness, double damping, double weight, const Eigen::Rotation3d& targetPose, bool usesYarpPorts = false);

        ~wOcraSegOrientationTaskManager();

        void setOrientation(const Eigen::Rotation3d& pos);

        // All Managers have this
        void activate();
        void deactivate();

        // For objective tasks
        void setStiffness(double stiffness);
        double getStiffness();
        void setDamping(double damping);
        double getDamping();
        void setWeight(double weight);
        double getWeight();

        // Task error
        Eigen::VectorXd getTaskError();

        // Yarp related:
        virtual const double * getCurrentState();
        virtual std::string getTaskManagerType();
        virtual bool checkIfActivated();


    private:
        // wocra::wOcraTask*              task;
        const std::string&              segmentName;

        ocra::OrientationFeature*        feat;
        ocra::SegmentFrame*              featFrame;
        ocra::OrientationFeature*        featDes;
        ocra::TargetFrame*               featDesFrame;

//        Eigen::Displacementd            _poseDes;

        void _init(Eigen::Rotation3d refOrientation_LocalFrame, double stiffness, double damping, double weight);
};

}

#endif // wOcraSEGORIENTATIONTASKMANAGER_H
