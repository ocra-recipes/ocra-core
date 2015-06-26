#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "wocra/wOcraController.h"
#include "wocra/Model.h"

#include <iterator>

namespace wocra
{
    wOcraTaskSequenceBase::~wOcraTaskSequenceBase()
    {
    }

    void wOcraTaskSequenceBase::init(wOcraController& ctrl, wocra::wOcraModel& model)
    {
        doInit(ctrl, model);
    }

    void wOcraTaskSequenceBase::update(double time, wocra::wOcraModel& state, void** args)
    {
        // Update all of the individual task managers:
        if (!taskManagers.empty()) {
            for (tmIterator it = taskManagers.begin(); it != taskManagers.end(); it++){
                it->second->refreshPorts();
            }
        }else{std::cout << "[WARNING] No tasks detected!!!" << std::endl;}

        // Run the custom doUpdate function of the cpp Sequences for hard coded control logic:
        doUpdate(time, state, args);
    }

    bool wOcraTaskSequenceBase::addTaskManager(std::string keyValue, wOcraTaskManagerBase* newTaskManager)
    {
        if (newTaskManager==NULL) {
            std::cout << "WARNING: [addTaskManager] The newTaskManager pointer you passed was empty." << std::endl;
            return false;
        }

        //Check if key already exists... If not, .find() will return a iterator to the end of the map.

        if (taskManagers.find(keyValue) == taskManagers.end()) {
            taskManagers[keyValue] = newTaskManager;
            return true;
        }
        else{
            std::cout << "WARNING: [addTaskManager] The key value you passed already exists. Cannot overwrite tasks, please remove tasks before replacing them." << std::endl;
            return false;
        }
    }

    bool wOcraTaskSequenceBase::removeTaskManager(std::string keyValue)
    {
        if (taskManagers.find(keyValue) != taskManagers.end()) {
            // taskManagers.erase
            //TODO: Implement some sort of task phase out trigger. If we just remove the task then it could cause problems.
            return true;
        }
        else{
            std::cout << "ERROR: [removeTaskManager] The task key you passed does not exist." << std::endl;
            return false;
        }
    }

}
