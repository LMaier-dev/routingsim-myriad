

#include "GaussMarkovMobility.h"
#include "BaseTypes.h"
#include "clog.h"

namespace routingsim {

Define_Module(GaussMarkovMobility);

void GaussMarkovMobility::initialize() {
    MobilityBase::initialize();
    double beta = par("beta").doubleValue();

    alpha = exp(-beta * updateInterval);
    ASSERT(0 <= alpha && alpha <= 1);

    speedMean = par("speedMean");
    speedVariance = par("speedVariance");
    headingMean = par("headingMean");
    headingVariance = par("headingVariance");
    borderMarginX = par("borderMarginX");
    borderMarginY = par("borderMarginY");

    ASSERT(2*borderMarginX < borderMax.x - borderMin.x);
    ASSERT(2*borderMarginY < borderMax.y - borderMin.y);

    targetPosition = lastPosition;

    EV_DEBUG << "initialize gauss-markov with memory level alpha: " << alpha << endl;
}

void GaussMarkovMobility::move() {
    // Update position
    handleBorderMargin();
    updateDirection();
    targetPosition = lastPosition + velocity;
    lastPosition = targetPosition;
}

void GaussMarkovMobility::updateDirection() {
    // Speed
    double speed = alpha * velocity.distance() + (1 - alpha) * speedMean + speedVariance * sqrt(1 - alpha*alpha) * normal(0, 1, MOBILITY_RNG);
    // Heading
    heading = Rad2D(alpha * heading.getRadius() + (1 - alpha) * headingMean + headingVariance * sqrt(1 - alpha*alpha) * normal(0, 1, MOBILITY_RNG));
    // Velocity
    velocity = heading.toVector() * speed;
}

void GaussMarkovMobility::handleBorderMargin() {
    bool inLeftMargin, inRightMargin, inTopMargin, inBottomMargin;
    inLeftMargin = inRightMargin = inTopMargin = inBottomMargin = false;
    double leftMargin = borderMin.x + borderMarginX;
    double rightMargin = borderMax.x - borderMarginX;
    double topMargin = borderMax.y - borderMarginY;
    double bottomMargin = borderMin.y + borderMarginY;

    inLeftMargin = lastPosition.x <= leftMargin;
    inRightMargin = lastPosition.x >= rightMargin;
    inTopMargin = lastPosition.y >= topMargin;
    inBottomMargin = lastPosition.y <= bottomMargin;

    // Change mean direction based on position
    if (inTopMargin and inLeftMargin) {
        headingMean = 315;
    } else if (inTopMargin and inRightMargin) {
        headingMean = 225;
    } else if (inBottomMargin and inRightMargin) {
        headingMean = 135;
    } else if (inBottomMargin and inLeftMargin) {
        headingMean = 45;
    } else if (inLeftMargin) {
        headingMean = 0;
    } else if (inTopMargin) {
        headingMean = 270;
    } else if (inRightMargin) {
        headingMean = 180;
    } else if (inBottomMargin) {
        headingMean = 90;
    }
}

};
