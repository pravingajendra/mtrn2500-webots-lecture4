#include "EPuckGreenPlusRobot.hpp"

#include <cmath>
#include <iostream>

EPuckGreenPlusRobot::EPuckGreenPlusRobot() 
    : mGPS {getGPS("gps")}
    , mDisplay1 {getDisplay("display1")} 
    , mDP1Width {mDisplay1->getWidth()}
    , mDP1Height {mDisplay1->getHeight()}
    , mCompass {getCompass("compass")}
    , mLidar {getLidar("lidar")}
    , mDisplay2 {getDisplay("display2")}
    , mDP2Width {mDisplay2->getWidth()}
    , mDP2Height {mDisplay2->getHeight()}
    {
        mGPS->enable(TIME_STEP);
        mDP1Width = mDisplay1->getWidth();
        mDP1Height = mDisplay1->getHeight();
        mCompass->enable(TIME_STEP);
        mLidar->enable(TIME_STEP);
        mLidar->enablePointCloud();


}
void EPuckGreenPlusRobot::run() {
    while (step(TIME_STEP) != -1) {
        telecontrol();

        readGPS();
        displayTrajectory();

        readCompass();
        readLidar();
        displayMap();

        std::cout << std::endl;
    }
}

void EPuckGreenPlusRobot::readGPS() {
    const double* gpsValues {mGPS->getValues()};
    mPosition = {gpsValues[0], gpsValues[1]};
    std::cout << "X: " << mPosition.first 
              << "  Y: " << mPosition.second;
}

void EPuckGreenPlusRobot::displayTrajectory() const {
    const double x {(((WORLD_WIDTH / 2) + mPosition.first) / WORLD_WIDTH) * mDP1Width};
    const double y {(((WORLD_HEIGHT / 2) - mPosition.second) / WORLD_HEIGHT) * mDP1Height};
    mDisplay1->drawPixel(x, y);
}

void EPuckGreenPlusRobot::readCompass() {
    const double* compassValues {mCompass->getValues()};
    const double rad {atan2(compassValues[1], compassValues[0])};
    mHeading = (rad / 3.14159265359) * 180.0;
    std::cout << "  Heading: " << mHeading;
}

void EPuckGreenPlusRobot::readLidar() {
    const webots::LidarPoint* lidarPoints {mLidar->getLayerPointCloud(0)};
    mPointCloud.clear();
    for (int i {}; i < mLidar->getNumberOfPoints(); i++) {
        mPointCloud.push_back({lidarPoints[i].x, lidarPoints[i].y});
    }
}

void EPuckGreenPlusRobot::displayMap() const {
    for (const auto& point : mPointCloud) {
        const double xLocal {point.first};
        const double yLocal {point.second};

        const double headingRad {3.14159265359 * mHeading / 180.0};

        const double xWorld {mPosition.first + xLocal * sin(headingRad) - yLocal * cos(headingRad)};
        const double yWorld {mPosition.second + xLocal * cos(headingRad) + yLocal * sin(headingRad)};

        const double x {(((WORLD_WIDTH / 2) + xWorld) / WORLD_WIDTH) * mDP2Width};
        const double y {(((WORLD_HEIGHT / 2) - yWorld) / WORLD_HEIGHT) * mDP2Height};
        mDisplay2->drawPixel(x, y);
    }
}
