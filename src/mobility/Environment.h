//
// Created by s_schaut on 23.03.24.
//

#ifndef ROUTINGSIM_EVIROMENT_H
#define ROUTINGSIM_EVIROMENT_H

#include <omnetpp.h>
#include "../foundation/BaseTypes.h"

namespace routingsim {

    class Environment : public cSimpleModule {
    protected:
        void initialize(int stage) override;
        Coord2D borderMin;
        Coord2D borderMax;
    public:
        bool isOutside(Coord2D) const;
        Coord2D getBorderMin();
        Coord2D getBorderMax();

        Coord2D getRandomPosition();
    };

} // routingsim

#endif //ROUTINGSIM_EVIROMENT_H
