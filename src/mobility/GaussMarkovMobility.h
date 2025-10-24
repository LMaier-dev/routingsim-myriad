#ifndef ROUTINGSIM_GAUSSMARKOV_H
#define ROUTINGSIM_GAUSSMARKOV_H

#include "MobilityBase.h"
#include "simtime_t.h"

namespace routingsim {

    class GaussMarkovMobility : public MobilityBase {
        private:
            // Gauss-Markov Mobility Memory Level
            double alpha;
            double one_minus_alpha;
            double sqrt_one_minus_alpha_square;
            double speedMean;
            double speedStandardDeviation;
            double headingMean;
            double headingStandardDeviation;
            simtime_t positionUpdateInterval; // when to change the velocity and direction
            simtime_t lastPositionUpdate;

            double topMargin, rightMargin, bottomMargin, leftMargin;

            void updateDirection();
            void handleBorderMargin();
        protected:
            // Move the node to the targetPosition and update the lastPosition of the node.
            void move() override;
            //
            void handleIfOutside();
        public:
            // Initialize the mobility model.
            void initialize() override;
            // Returns a random position inside the margins
            Coord2D getRandomPosition();
    };
}

#endif //ROUTINGSIM_GAUSSMARKOV_H
