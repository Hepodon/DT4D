#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/drivetrains4dummies.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"

using namespace pros;
using namespace std;

PIDsettings pids(100, 5, 15, 100);

MotorGroup aright({5, 10}, v5::MotorGears::green,
                  v5::MotorEncoderUnits::degrees);
MotorGroup aleft({-1, -6}, v5::MotorGears::green,
                 v5::MotorEncoderUnits::degrees);

Imu inertial(12);

Controller userinput(E_CONTROLLER_MASTER);

DTSettings saettings(10, 4, 1, 9, 1, 2, 2, 2, 2);
DTSettingsNT settings(10, 4, 1, 9, 1, 2, 2, 2, 2, 100, 100);

Drivetrain4Dummies bot(pids, aleft, -5, aright, 5, saettings, inertial);

Drivetrain4DummiesNT botNT(pids, aleft, -5, aright, 5, settings, inertial,
                           userinput);

void initialize() {
  aleft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  aright.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {}

void competition_initialize() {}

void autonomous() { botNT.move_DriveFor(true, Fwd, 12); }

int power;
int turn;
int leftPower = 0;
int rightPower = 0;

void opcontrol() {
  while (true) {
    power = userinput.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    turn = userinput.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    power = power = 0 ? -1 : power;
    turn = turn = 0 ? -1 : turn;

    leftPower = applySlew(leftPower, power + turn, 10);
    rightPower = applySlew(rightPower, power - turn, 10);

    aleft.move(leftPower);
    aright.move(rightPower);

    while (userinput.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      leftPower = 0;
      rightPower = 0;
      aleft.brake();
      aright.brake();
    }

    pros::delay(150);
  }
}