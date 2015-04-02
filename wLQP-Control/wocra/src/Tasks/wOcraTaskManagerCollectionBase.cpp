#include "wocra/Tasks/wOcraTaskManagerCollectionBase.h"
#include "wocra/wOcraController.h"
#include "wocra/Model.h"

namespace wocra
{
    wOcraTaskManagerCollectionBase::~wOcraTaskManagerCollectionBase()
    {
    }

    void wOcraTaskManagerCollectionBase::init(wOcraController& ctrl, wocra::wOcraModel& model)
    {
        doInit(ctrl, model);
    }

    void wOcraTaskManagerCollectionBase::update(double time, wocra::wOcraModel& state, void** args)
    {
        doUpdate(time, state, args);
    }
}
