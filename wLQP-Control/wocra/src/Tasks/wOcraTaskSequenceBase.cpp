#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "wocra/wOcraController.h"
#include "wocra/Model.h"

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
        doUpdate(time, state, args);
    }
}
