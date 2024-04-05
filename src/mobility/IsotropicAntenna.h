//
// Created by s_schaut on 05.04.24.
//

#ifndef ROUTINGSIM_ISOTROPICANTENNA_H
#define ROUTINGSIM_ISOTROPICANTENNA_H

#include "AntennaBase.h"

namespace routingsim {

class IsotropicAntenna : public AntennaBase {
public:
  double pathloss(double distance) override;
};

} // namespace routingsim

#endif // ROUTINGSIM_ISOTROPICANTENNA_H
