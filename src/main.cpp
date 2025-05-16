#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/drivetrains4dummies.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"

using namespace pros;
using namespace std;
using namespace DT4D;

DT4D::PIDsettings pids(100, 5, 15, 100);

MotorGroup aright({5, 10}, v5::MotorGears::green,
                  v5::MotorEncoderUnits::degrees);
MotorGroup aleft({-1, -6}, v5::MotorGears::green,
                 v5::MotorEncoderUnits::degrees);

Imu inertial(12);

Controller userinput(E_CONTROLLER_MASTER);

DT4D::DTSettings saettings(10, 4, 1, 9, 1, 2, 2, 2, 2);
DT4D::DTSettingsNT settings(10, 4, 1, 9, 1, 2, 2, 2, 2, 100, 100);

DT4D::Drivetrain4Dummies bot(pids, aleft, -5, aright, 5, saettings, inertial);

DT4D::Drivetrain4DummiesNT botNT(pids, aleft, -5, aright, 5, settings, inertial,
                                 userinput);

void initialize() {
  aleft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  aright.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  botNT.CALIBRATE();
  botNT.move_TurnFor(PID, Left, 90);
  botNT.move_DriveFor(PID, Fwd, 12);
  botNT.stop();}

void opcontrol() {
  int power;
  int turn;
  int leftPower = 0;
  int rightPower = 0;
  while (true) {
    power = userinput.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    turn = userinput.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    if (power != 0) {
      leftPower =
          DT4D::applySlew(leftPower, power + turn, 10 + (10.0 * power / 100));
      rightPower =
          DT4D::applySlew(rightPower, power - turn, 10 + (10.0 * power / 100));
    } else {
      leftPower = DT4D::applySlew(leftPower, power + turn, 25);
      rightPower = DT4D::applySlew(rightPower, power - turn, 25);
    }
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