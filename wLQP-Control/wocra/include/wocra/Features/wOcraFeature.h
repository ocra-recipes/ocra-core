/**
 * \file wOcraFeature.h
 * \author Joseph Salini
 *
 * \brief Define the partial state feature that can manage partial state instances.
 */

#ifndef __wOcraFEATURE_H__
#define __wOcraFEATURE_H__

// OCRA INCLUDES
#include "ocra/control/Feature.h"


// WOCRA INCLUDES
#include "wocra/Features/wOcraPartialState.h"


namespace wocra
{

/** \addtogroup feature
 * \{
 */

/** \brief A partial state feature.
 *
 * This class is greatly inspired from the \b FullStateFeature class defined in the ocra framework.
 */
class PartialStateFeature: public ocra::Feature
{
public:
    PartialStateFeature(const std::string& name, const PartialState& state);

    int getDimension() const;

    const Eigen::MatrixXd& getSpaceTransform() const;

    const Eigen::VectorXd& computeEffort(const Feature& featureDes) const;
    const Eigen::VectorXd& computeAcceleration(const Feature& featureDes) const;
    const Eigen::VectorXd& computeError(const Feature& featureDes) const;
    const Eigen::VectorXd& computeErrorDot(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeJacobian(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeProjectedMass(const Feature& featureDes) const;
    const Eigen::MatrixXd& computeProjectedMassInverse(const Feature& featureDes) const;

    const Eigen::VectorXd& computeEffort() const;
    const Eigen::VectorXd& computeAcceleration() const;
    const Eigen::VectorXd& computeError() const;
    const Eigen::VectorXd& computeErrorDot() const;
    const Eigen::MatrixXd& computeJacobian() const;
    const Eigen::MatrixXd& computeProjectedMass() const;
    const Eigen::MatrixXd& computeProjectedMassInverse() const;

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
};

/** \} */ // end group feature


} //end of namespace wocra

#endif
