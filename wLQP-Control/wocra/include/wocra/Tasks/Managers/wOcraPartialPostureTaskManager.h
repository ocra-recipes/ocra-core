#ifndef wOcraPARTIALPOSTURETASKMANAGER_H
#define wOcraPARTIALPOSTURETASKMANAGER_H

#include "wocra/Tasks/wOcraTaskManagerBase.h"
#include "ocra/control/Model.h"
#include "wocra/Tasks/wOcraTask.h"
#include "wocra/wOcraController.h"
#include "wocra/Features/wOcraFeature.h"

#include <Eigen/Dense>

namespace wocra
{

/** \brief Task Manager for partal joint posture task
 *
 */
class wOcraPartialPostureTaskManager : public wOcraTaskManagerBase
{
    public:
        wOcraPartialPostureTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, int fullStateType, Eigen::VectorXi& dofIndices, double stiffness, double damping, double weight);

        wOcraPartialPostureTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, int fullStateType, Eigen::VectorXi& dofIndices, double stiffness, double damping, double weight,
            Eigen::VectorXd& init_q);

        ~wOcraPartialPostureTaskManager();

        void setPosture(Eigen::VectorXd& q);
        void setPosture(Eigen::VectorXd& q, Eigen::VectorXd& qdot, Eigen::VectorXd& qddot);
/*
        void setStiffnessDamping(double stiffness, double damping);
*/
        void setWeight(double weight);
        double getWeight();
        void activate();
        void deactivate();

        // Yarp related:
        virtual const double * getCurrentState();
        virtual std::string getTaskManagerType();
        virtual bool checkIfActivated();

    private:
        wocra::wOcraTask*                      task;
        wocra::PartialStateFeature*           feat;
        wocra::PartialModelState*             featState;

        wocra::PartialStateFeature*           featDes;
        wocra::PartialTargetState*            featDesState;

/*
        Eigen::Vector3d                 _posDes;
        Eigen::Vector3d                 _velDes;
        Eigen::Vector3d                 _accDes;
*/

        void _init(int fullStateType, VectorXi& dofIndices, double stiffness, double damping, double weight);
};

}

#endif // wOcraFULLPOSTURETASKMANAGER_H
