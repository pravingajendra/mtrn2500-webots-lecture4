#include "EPuckGreenPlusRobot.hpp"

EPuckGreenPlusRobot::EPuckGreenPlusRobot() 
    : mGPS {getGPS("gps")}
    , mDisplay1 {getDisplay("display1")} 
    {
        mGPS->enable(TIME_STEP);
        mDP1Width = mDisplay1->getWidth();
        mDP1Height = mDisplay1->getHeight();

}
void EPuckGreenPlusRobot::run() {
    while (step(TIME_STEP) != -1) {
        telecontrol();

        readGPS();
        displayTrajectory();
    }
}

void EPuckGreenPlusRobot::readGPS() {
    const double* gpsValues {mGPS->getValues()};
    mPosition = {gpsValues[0], gpsValues[1]};
    std::cout << "X: " << mPosition.first 
              << "  Y: " << mPosition.second << std::endl;
}

void EPuckGreenPlusRobot::displayTrajectory() const {
    const double x {(((WORLD_WIDTH / 2) + mPosition.first) / WORLD_WIDTH) * mDP1Width};
    const double y {(((WORLD_HEIGHT / 2) + mPosition.second) / WORLD_HEIGHT) * mDP1Height};
    mDisplay1->drawPixel(x, y);
}
