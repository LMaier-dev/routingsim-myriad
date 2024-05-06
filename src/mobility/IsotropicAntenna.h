
#ifndef ROUTINGSIM_ISOTROPICANTENNA_H
#define ROUTINGSIM_ISOTROPICANTENNA_H

#include <cmath>
#include "AntennaBase.h"

namespace routingsim {

    class IsotropicAntenna : public AntennaBase {
    public:
        double pathloss(double distance) override;

        double distance_from_pathloss(double pathloass) override;

        double calcDatarate(double distance) override;
    };

} // namespace routingsim

#endif // ROUTINGSIM_ISOTROPICANTENNA_H
