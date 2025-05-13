#include "main.h"
#include "pros/drivetrains4dummies.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"

using namespace pros;
using namespace std;

DTSettings settings(12, 12, 12, 1, 12);

Controller controller(E_CONTROLLER_MASTER);
Imu inertial(20);

MotorGroup leftMotors({1, 2, 3});
MotorGroup rightMotors({-4, -5, -6});

Drivetrain4Dummies drivetrain(leftMotors, 13, rightMotors, 12, settings,
                              inertial);

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {}