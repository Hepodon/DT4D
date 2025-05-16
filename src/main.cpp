#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/drivetrains4dummies.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"

using namespace pros;
using namespace std;

PIDsettings pids(100, 5, 15, 100);

MotorGroup aleft({5, 10}, v5::MotorGears::green,
                 v5::MotorEncoderUnits::degrees);
MotorGroup aright({-1, -6}, v5::MotorGears::green,
                  v5::MotorEncoderUnits::degrees);

Imu inertial(12);

Controller userinput(E_CONTROLLER_MASTER);

DTSettings saettings(10, 4, 1, 9, 1, 2, 2, 2, 2);
DTSettingsNT settings(10, 4, 1, 9, 1, 2, 2, 2, 2, 100, 100);

Drivetrain4Dummies bot(pids, aleft, -5, aright, 5, saettings, inertial);

Drivetrain4DummiesNT botNT(pids, aleft, -5, aright, 5, settings, inertial,
                           userinput);

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() { botNT.move_DriveFor(true, Fwd, 12); }

void opcontrol() {
  while (true) {
    int fwdPower = userinput.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turnPower = userinput.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    aleft.move(fwdPower - turnPower);
    aright.move(fwdPower + turnPower);
    delay(20);
  }
}