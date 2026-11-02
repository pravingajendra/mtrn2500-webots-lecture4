#pragma once

#include <utility>

#include <webots/GPS.hpp>
#include <webots/Display.hpp>

#include "EPuckGreenRobot.hpp"

constexpr double WORLD_WIDTH {1.0};
constexpr double WORLD_HEIGHT {1.0};

class EPuckGreenPlusRobot : public EPuckGreenRobot {
public:
    EPuckGreenPlusRobot();
    void run();
protected:
    void readGPS();
    void displayTrajectory() const;
private:
    webots::GPS* mGPS {};
    std::pair<double, double> mPosition {};

    webots::Display* mDisplay1 {};
    int mDP1Width {};
    int mDP1Height {};
};