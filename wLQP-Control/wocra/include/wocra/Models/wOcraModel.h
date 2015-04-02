/**^M
 * \file Model.h^M
 * \author Darwin Lau
 *^M
 * \brief Inherited ocra::Model with added functionality and template
 */
#ifndef __WOCRAMODEL_H__
#define __WOCRAMODEL_H__

#include <iostream>
#include <stdlib.h> // to get relative path of file
#include <dlfcn.h> // To load shared libraries
#include <ocra/control/Model.h>

namespace wocra{

class wOcraModel: public ocra::Model
{
public:

//===========================Constructor/Destructor===========================//
    wOcraModel(const std::string& name, int ndofs, bool freeRoot);
    virtual ~wOcraModel();

//=============================General functions==============================//
    int getDofIndex(const std::string& name) const;
    const std::string& getDofName(int index) const;
    const std::string DofName(const std::string& name) const;
    const std::string SegmentName(const std::string& name) const;

protected:
//============================Index name functions============================//
    virtual int                 doGetDofIndex       (const std::string& name) const;
    virtual const std::string&  doGetDofName        (int index) const;
    // Gets the references name for access within the controller
    virtual const std::string   doSegmentName       (const std::string& name) const;
    virtual const std::string   doDofName           (const std::string& name) const;

private:
};

}

#endif
