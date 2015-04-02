#ifndef wOcraTASKMANAGERCOLLECTIONBASE_H
#define wOcraTASKMANAGERCOLLECTIONBASE_H

// Includes the set of all possible task managers to make it easier to reference
#include "wocra/Tasks/wOcraTaskManagerBase.h"
#include "wocra/Tasks/Managers/wOcraCoMTaskManager.h"
#include "wocra/Tasks/Managers/wOcraContactTaskManager.h"
#include "wocra/Tasks/Managers/wOcraContactSetTaskManager.h"
#include "wocra/Tasks/Managers/wOcraFullPostureTaskManager.h"
#include "wocra/Tasks/Managers/wOcraPartialPostureTaskManager.h"
#include "wocra/Tasks/Managers/wOcraSegCartesianTaskManager.h"
#include "wocra/Tasks/Managers/wOcraSegOrientationTaskManager.h"
#include "wocra/Tasks/Managers/wOcraSegPoseTaskManager.h"

#include "wocra/Models/wOcraModel.h"

namespace wocra
{
    typedef std::map<std::string, wOcraTaskManagerBase*> TaskManagerDict;

    class wOcraTaskManagerCollectionBase
    {
        public:
            virtual ~wOcraTaskManagerCollectionBase();
            void init(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
            void update(double time, wocra::wOcraModel& state, void** args);
        protected: 
            virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model) = 0;
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args) = 0; 

            TaskManagerDict taskManagers;
    };
}

#endif // wOcraTASKMANAGERCOLLECTION_H
