
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
