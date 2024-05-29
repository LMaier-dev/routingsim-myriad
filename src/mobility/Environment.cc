
#include "Environment.h"

namespace routingsim {

Define_Module(Environment)

    void Environment::initialize() {
        borderMax = Coord2D(par("borderX"), par("borderY"));
        borderMin = Coord2D(0, 0);
    }

    Coord2D Environment::getRandomPosition() {
        return {uniform(borderMin.x, borderMax.x, 1), uniform(borderMin.y, borderMax.y, 1)};
    }

    bool Environment::isOutside(Coord2D position) const {
        return position.x < borderMin.x || position.x > borderMax.x
               || position.y < borderMin.y || position.y > borderMax.y;
    }
} // routingsim