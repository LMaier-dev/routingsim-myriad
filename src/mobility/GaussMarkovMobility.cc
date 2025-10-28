#include "GaussMarkovMobility.h"
#include "BaseTypes.h"
#include "clog.h"

namespace routingsim {

Define_Module(GaussMarkovMobility);

void GaussMarkovMobility::initialize() {
    MobilityBase::initialize();
    double beta = par("beta").doubleValue();

    positionUpdateInterval= SimTime(2500,SIMTIME_MS);
    alpha = exp(-beta * positionUpdateInterval.dbl());
    ASSERT(0 <= alpha && alpha <= 1);
    EV_DEBUG << "alpha: " << alpha << ", beta: " << beta << endl;
    sqrt_one_minus_alpha_square= sqrt(1.0 - alpha*alpha);
    one_minus_alpha= 1.0-alpha;
    speedMean = par("speedMean");
    speedStandardDeviation = par("speedStandardDeviation");
    headingMean = par("headingMean");
    headingStandardDeviation = par("headingStandardDeviation");
    double borderMarginX = par("borderMarginX");
    double borderMarginY = par("borderMarginY");

    ASSERT(2*borderMarginX < borderMax.x - borderMin.x);
    ASSERT(2*borderMarginY < borderMax.y - borderMin.y);

    leftMargin = borderMin.x + borderMarginX;
    rightMargin = borderMax.x - borderMarginX;
    topMargin = borderMax.y - borderMarginY;
    bottomMargin = borderMin.y + borderMarginY;
    EV_DEBUG << "Border Margin: (" << leftMargin << "," << bottomMargin << "," << rightMargin << "," << topMargin << ")" << endl;

    lastPosition = getRandomPosition();
    targetPosition = lastPosition;
    lastPositionUpdate = simTime();

    EV_DEBUG << "initialize gauss-markov with memory level alpha: " << alpha << endl;
}

Coord2D GaussMarkovMobility::getRandomPosition() {
    // Chose random position inside margins
    return {uniform(leftMargin, rightMargin, MOBILITY_RNG), uniform(bottomMargin, topMargin, MOBILITY_RNG)};
}

void GaussMarkovMobility::move() {
    // Update position
    EV_DEBUG << "MOVE" << endl;
    double oldheadingMean= headingMean;
    bool in_margin = handleBorderMargin();
    // update/change direction and speed if next interval has been reached
    if ( ((simTime() - lastPositionUpdate) >= positionUpdateInterval) or in_margin ) {
        updateDirection();
        lastPositionUpdate = simTime();
    }
    targetPosition = lastPosition + velocity;
    lastPosition = targetPosition;
}


void GaussMarkovMobility::updateDirection() {
    EV_DEBUG << "Old Velocity: " << velocity.toString() << " Old Heading: " << heading.getRadius() << " Old Heading Mean: " << headingMean << endl;
    // Speed
    double newspeed = alpha * velocity.distance() + one_minus_alpha * speedMean + speedStandardDeviation * sqrt_one_minus_alpha_square * normal(0.0, 1.0, MOBILITY_RNG);
    // Heading
    double rand_heading = normal(0.0, 1.0, MOBILITY_RNG);
    double old_heading = heading.getRadius();
    heading = Rad2D(alpha * old_heading + one_minus_alpha * headingMean + headingStandardDeviation * sqrt_one_minus_alpha_square * rand_heading);
    EV_DEBUG << "heading: " << heading.getRadius() << " = " << alpha << " * " << old_heading << " + " << (1 - alpha ) <<  "*" << headingMean << " + " << headingStandardDeviation << "*" << sqrt_one_minus_alpha_square << " * " << rand_heading << endl;
    // Velocity
    velocity = heading.toVector() * newspeed;
    EV_DEBUG << "Velocity: " << velocity.toString() << " Speed: " << newspeed << "[m/s] Heading: " << heading.getRadius() << ", " << heading.toVector().toString() << " Heading Mean: " << headingMean << endl;
}


// returns true if inside margin, false otherwise
bool GaussMarkovMobility::handleBorderMargin() {
    bool inLeftMargin = lastPosition.x <= leftMargin;
    bool inRightMargin = lastPosition.x >= rightMargin;
    bool inTopMargin = lastPosition.y >= topMargin;
    bool inBottomMargin = lastPosition.y <= bottomMargin;

    EV_DEBUG << "In margin (" << inTopMargin << "," << inRightMargin << "," << inBottomMargin << "," << inLeftMargin << ")" << endl;

    // Change mean direction based on position
    if (inTopMargin and inLeftMargin) {
        headingMean = 315; // Head down right
    } else if (inTopMargin and inRightMargin) {
        headingMean = 225; // Head down left
    } else if (inBottomMargin and inRightMargin) {
        headingMean = 135; // Head up left
    } else if (inBottomMargin and inLeftMargin) {
        headingMean = 45; // Head up right
    } else if (inLeftMargin) {
        headingMean = 0; // Head right
    } else if (inTopMargin) {
        headingMean = 270; // Head down
    } else if (inRightMargin) {
        headingMean = 180; // Head left
    } else if (inBottomMargin) {
        headingMean = 90; // Head up
    } else { // not in margin
        //ASSERT(not inTopMargin and not inRightMargin and not inBottomMargin and not inLeftMargin);
        return false;
    }
    //ASSERT(inTopMargin or inRightMargin or inBottomMargin or inLeftMargin);
    // need fast turn around, so set also "oldheading" to new value
    heading = headingMean;
    velocity = heading.toVector() * velocity.distance();

    return true;
}

} // end namespace
