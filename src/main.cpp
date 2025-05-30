#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/drivetrains4dummies.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"

using namespace pros;
using namespace std;

DT4D::PIDsettings pids(100, 5, 15, 100);

Rotation hor1(-15);
Rotation vert1(11);
Rotation vert2(-12);

MotorGroup aright({1, 2, 3}, v5::MotorGears::blue,
                  v5::MotorEncoderUnits::degrees);
MotorGroup aleft({-20, -19, -7}, v5::MotorGears::green,
                 v5::MotorEncoderUnits::degrees);

Imu inertial(12);

DT4D::Trackingwheel LEFTVERT(vert1, 1, -5, 2);
DT4D::Trackingwheel RIGHTVERT(vert2, 1, 5, 2);
DT4D::Trackingwheel BACKHOR(hor1, 1, 0, 2);

DT4D::OdomSensors Sensors(LEFTVERT, RIGHTVERT, BACKHOR, inertial);

Controller userinput(E_CONTROLLER_MASTER);

DT4D::DTSettings saettings(10, 4, 1, 9, 1, 2, 2, 2, 2);
DT4D::DTSettingsNT settings(10, 4, 1, 9, 1, 2, 2, 2, 2, 100, 100);

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
  botNT.stop();
}

void opcontrol() {

  float driveValue = 0;
  float turnValue = 0;

  int leftPower = 0;
  int rightPower = 0;

  const float driveSlewRate = 8;
  const float turnSlewRate = 16;

  while (true) {
    float rawDrive = userinput.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
    float rawTurn = userinput.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

    driveValue = DT4D::applySlew(driveValue, rawDrive, driveSlewRate);
    turnValue = DT4D::applySlew(turnValue, rawTurn, turnSlewRate);

    leftPower = driveValue + turnValue;
    rightPower = driveValue - turnValue;

    aleft.move(leftPower);
    aright.move(rightPower);

    if (userinput.get_digital(E_CONTROLLER_DIGITAL_R1)) {
      aleft.brake();
      aright.brake();
      driveValue = 0;
      turnValue = 0;
    }

    delay(20);
  }
}
