
//
// This module inherits from the AntennaBase module and is a concrete antenna model that implements the isotropic antenna model
// It overwrites the methods for calculating the path loss and the data rate in order to take into account the specific properties of the isotropic antenna model
//

#ifndef ROUTINGSIM_ISOTROPICANTENNA_H
#define ROUTINGSIM_ISOTROPICANTENNA_H

#include "AntennaBase.h"
#include <cmath>

namespace routingsim {

class IsotropicAntenna : public AntennaBase {
public:
  DB pathloss(double distance) override;

  double distance_from_pathloss(DB pathloass) override;

  double calcDatarate(double distance) override;
};

} // namespace routingsim

#endif // ROUTINGSIM_ISOTROPICANTENNA_H
