#include "main.h"
#include "pros/drivetrains4dummies.hpp"
#include "pros/misc.hpp"

using namespace pros;
using namespace std;

MotorGroup aleft({1, 2, 3});
MotorGroup aright({-4, -5, -6});

Imu inertial(12);

Controller userinput(E_CONTROLLER_MASTER);

DTSettings saettings(10, 4, 1, 9, 1, 2, 2, 2, 2);
DTSettingsNT settings(10, 4, 1, 9, 1, 2, 2, 2, 2, 100, 100);

Drivetrain4Dummies bot(aleft, -5, aright, 5, saettings, inertial);

Drivetrain4DummiesNT botNT(aleft, -5, aright, 5, settings, inertial, userinput);

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() { botNT.move_TurnFor(Left, 90, 12, 12, true); }