//
// Created by s_schaut on 23.03.24.
//

#include <omnetpp.h>
#include "Environment.h"

namespace routingsim {

    void Environment::initialize(int stage) {
        cComponent::initialize(stage);
        borderMax = Coord2D(par("borderX"), par("borderY"));
        borderMin = Coord2D(0, 0);
    }

    Coord2D Environment::getRandomPosition() {
        return {uniform(borderMin.x, borderMax.x), uniform(borderMin.y, borderMax.y)};
    }

    bool Environment::isOutside(Coord2D position) const {
        return position.x < borderMin.x || position.x > borderMax.x
               || position.y < borderMin.y || position.y > borderMax.y;
    }

    Coord2D Environment::getBorderMin() {
        return borderMin;
    }

    Coord2D Environment::getBorderMax() {
        return borderMax;
    }
} // routingsim