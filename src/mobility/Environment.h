//
// This class is responsible for managing the environment of the simulation.
// The environment is defined by a rectangular area with a four points to define the borders (borderMin and borderMax).
// The class provides methods to handle the environment, such as checking if a position is outside the environment and
// generating random positions inside the environment.
//

#ifndef ROUTINGSIM_EVIROMENT_H
#define ROUTINGSIM_EVIROMENT_H

#include "../foundation/BaseTypes.h"

namespace routingsim {

    class Environment : public cSimpleModule {
    protected:
        // initialize the environment with the borders
        void initialize() override;

        // Environment borders
        Coord2D borderMin;
        Coord2D borderMax;

        // Field noise for the environment
        double systemNoise;

    public:
        // Check if a position is outside the environment
        bool isOutside(Coord2D) const;

        // Get a random position inside the environment using the uniform distribution
        Coord2D getRandomPosition();
    };

} // routingsim

#endif //ROUTINGSIM_EVIROMENT_H
