

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
    EV_DEBUG << "alpha: " << alpha << ", beta: " << beta << endl;

    speedMean = par("speedMean");
    speedVariance = par("speedVariance");
    headingMean = par("headingMean");
    headingVariance = par("headingVariance");
    double borderMarginX = par("borderMarginX");
    double borderMarginY = par("borderMarginY");

    ASSERT(2*borderMarginX < borderMax.x - borderMin.x);
    ASSERT(2*borderMarginY < borderMax.y - borderMin.y);

    leftMargin = borderMin.x + borderMarginX;
    rightMargin = borderMax.x - borderMarginX;
    topMargin = borderMax.y - borderMarginY;
    bottomMargin = borderMin.y + borderMarginY;
    EV_DEBUG << "Border Margin: (" << topMargin << "," << rightMargin << "," << bottomMargin << "," << leftMargin << ")" << endl;

    lastPosition = getRandomPosition();
    targetPosition = lastPosition;

    EV_DEBUG << "initialize gauss-markov with memory level alpha: " << alpha << endl;
}

Coord2D GaussMarkovMobility::getRandomPosition() {
    return {uniform(leftMargin, rightMargin, 1), uniform(bottomMargin, topMargin, 1)};
}

void GaussMarkovMobility::move() {
    // Update position
    EV_DEBUG << "MOVE" << endl;
    handleBorderMargin();
    updateDirection();
    targetPosition = lastPosition + velocity;
    lastPosition = targetPosition;
}

void GaussMarkovMobility::updateDirection() {
    EV_DEBUG << "Old Velocity: " << velocity.toString() << " Old Heading: " << heading.getRadius() << " Old Heading Mean: " << Rad2D(headingMean).toVector().toString() << endl;
    // Speed
    double speed = alpha * velocity.distance() + (1 - alpha) * speedMean + speedVariance * sqrt(1 - alpha*alpha) * normal(0, 1, MOBILITY_RNG);
    // Heading
    double rand_heading = normal(0, 1, MOBILITY_RNG);
    double old_heading = heading.getRadius();
    heading = Rad2D(alpha * heading.getRadius() + (1 - alpha) * headingMean + headingVariance * sqrt(1 - alpha*alpha) * rand_heading);
    EV_DEBUG << heading.getRadius() << " = " << alpha << " * " << old_heading << " + (1 -" << alpha << ") * " << headingMean << " + " << headingVariance << " * sqrt(1 -" << alpha*alpha << ") * " << rand_heading << endl;
    // Velocity
    velocity = heading.toVector() * speed;
    EV_DEBUG << "Velocity: " << velocity.toString() << " Heading: " << heading.getRadius() << ", " << heading.toVector().toString() << " Heading Mean: " << Rad2D(headingMean).toVector().toString() << endl;
}

void GaussMarkovMobility::handleBorderMargin() {
    bool inLeftMargin = lastPosition.x <= leftMargin;
    bool inRightMargin = lastPosition.x >= rightMargin;
    bool inTopMargin = lastPosition.y >= topMargin;
    bool inBottomMargin = lastPosition.y <= bottomMargin;

    EV_DEBUG << "In margin (" << inTopMargin << "," << inRightMargin << "," << inBottomMargin << "," << inLeftMargin << ")" << endl;

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
