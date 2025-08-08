

#ifndef ROUTINGSIM_GAUSSMARKOV_H
#define ROUTINGSIM_GAUSSMARKOV_H

#include "MobilityBase.h"

namespace routingsim {

    class GaussMarkovMobility : public MobilityBase {
        private:
            // Gauss-Markov Mobility Memory Level
            double alpha;
            double speedMean;
            double speedStandardDeviation;
            double headingMean;
            double headingStandardDeviation;

            double topMargin, rightMargin, bottomMargin, leftMargin;

            void updateDirection();
            void handleBorderMargin();
        protected:
            const int MOBILITY_RNG = 1;
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
