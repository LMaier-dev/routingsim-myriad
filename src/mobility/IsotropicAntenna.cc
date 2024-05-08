
#include "IsotropicAntenna.h"


namespace routingsim {

Define_Module(IsotropicAntenna);

DBm IsotropicAntenna::pathloss(double distance) {
    return 1 / pow(distance, 2);
}

double IsotropicAntenna::distance_from_pathloss(DBm pathloass) {
    return 1 / sqrt(pathloass);
}

double IsotropicAntenna::calcDatarate(double distance) {
    return bandwidth * log2(1 + (signalPower - pathloss(distance)) / systemNoise);
}

} // namespace routingsim
