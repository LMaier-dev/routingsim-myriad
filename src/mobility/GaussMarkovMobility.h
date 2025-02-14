

#ifndef ROUTINGSIM_GAUSSMARKOV_H
#define ROUTINGSIM_GAUSSMARKOV_H

#include "MobilityBase.h"

namespace routingsim {

    class GaussMarkovMobility : public MobilityBase {
        private:
            // Gauss-Markov Mobility Memory Level
            double alpha;
            double speedMean;
            double speedVariance;
            double headingMean;
            double headingVariance;
            double borderMarginX;
            double borderMarginY;

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
    };
}

#endif //ROUTINGSIM_GAUSSMARKOV_H
