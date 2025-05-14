// sup this is my files

#ifndef PROS_DRIVETRAINS4DUMMIES_HPP
#define PROS_DRIVETRAINS4DUMMIES_HPP

#include "math.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

/* ========================= ENUMS ========================= */

enum DirectionStraight { Forward = 1, Fwd = 1, Reverse = -1, Rev = -1 };
enum DirectionTurn { Left = 1, L = 1, Right = -1, R = -1 };
enum BrakingType { COAST = 0, BRAKE = 1, HOLD = 2, DEFAULT = 3 };
/* ========================================================== */

namespace pros {
inline namespace v5 {

class DTSettingsNT {

public:
  DTSettingsNT(int Wheelbase, int WheelDiameter, int GearRatio, double kP,
               double kI, double kD, double DkP, double DkI, double DkD,
               int driveVelocity = 50, int turnVelocity = 35)
      : _wheelbase(Wheelbase), _wheelDiameter(WheelDiameter),
        _gearRatio(GearRatio), _driveVelocity(driveVelocity),
        _turnVelocity(turnVelocity) {
    _kP = kP;
    _kI = kI;
    _kD = kD;

    D_kP = DkP;
    D_kI = DkI;
    D_kD = DkD;
  }

  int get_Wheelbase() const { return _wheelbase; }
  int get_WheelDiameter() const { return _wheelDiameter; }
  int get_GearRatio() const { return _gearRatio; }
  int get_driveVelocity() const { return _driveVelocity; }
  int get_turnVelocity() const { return _turnVelocity; }

  void set_Wheelbase(int Wheelbase) { _wheelbase = Wheelbase; }
  void set_WheelDiameter(int WheelDiameter) { _wheelDiameter = WheelDiameter; }
  void set_GearRatio(int GearRatio) { _gearRatio = GearRatio; }
  void set_driveVelocity(int driveVelocity) { _driveVelocity = driveVelocity; }
  void set_turnVelocity(int turnVelocity) { _turnVelocity = turnVelocity; }

  double get_kP() const { return _kP; }
  double get_kI() const { return _kI; }
  double get_kD() const { return _kD; }

  double get_DkP() const { return D_kP; }
  double get_DkI() const { return D_kI; }
  double get_DkD() const { return D_kD; }

  int _kP;
  int _kI;
  int _kD;

  int D_kP;
  int D_kI;
  int D_kD;

  int _wheelbase;
  int _wheelDiameter;
  int _gearRatio;
  int _driveVelocity;
  int _turnVelocity;
};
class DTSettings {

public:
  DTSettings(int Wheelbase, int WheelDiameter, int GearRatio, double kP,
             double kI, double kD, double DkP, double DkI, double DkD)
      : _wheelbase(Wheelbase), _wheelDiameter(WheelDiameter),
        _gearRatio(GearRatio) {
    _kP = kP;
    _kI = kI;
    _kD = kD;

    D_kP = DkP;
    D_kI = DkI;
    D_kD = DkD;
  }

  int get_Wheelbase() const { return _wheelbase; }
  int get_WheelDiameter() const { return _wheelDiameter; }
  int get_GearRatio() const { return _gearRatio; }

  void set_Wheelbase(int Wheelbase) { _wheelbase = Wheelbase; }
  void set_WheelDiameter(int WheelDiameter) { _wheelDiameter = WheelDiameter; }
  void set_GearRatio(int GearRatio) { _gearRatio = GearRatio; }

  double get_kP() const { return _kP; }
  double get_kI() const { return _kI; }
  double get_kD() const { return _kD; }

  double get_DkP() const { return D_kP; }
  double get_DkI() const { return D_kI; }
  double get_DkD() const { return D_kD; }

  int _kP;
  int _kI;
  int _kD;

  int D_kP;
  int D_kI;
  int D_kD;

  int _wheelbase;
  int _wheelDiameter;
  int _gearRatio;
};
class Drivetrain4Dummies {

public:
  Drivetrain4Dummies(MotorGroup &_leftMotors, int leftDistance,
                     MotorGroup &_rightMotors, int rightDistance,
                     DTSettings &_settings, Imu &_inertial);
  MotorGroup &_leftMotors;
  MotorGroup &_rightMotors;
  Imu &_inertial;
  DTSettings &_settings;

  int _leftDistance;
  int _rightDistance;

  int _x;
  int _y;
  int _theta;

  void CALIBRATE(int x = 0, int y = 0, int theta = 0) {
    _inertial.reset();
    while (_inertial.is_calibrating()) {
      pros::delay(20);
    }
    _leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    _rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    _x = x;
    _y = y;
    _theta = theta;
  }
  void reset() {
    _x = 0;
    _y = 0;
    _theta = 0;
    _inertial.tare();
  }

  int getX() const { return _x; }
  int getY() const { return _y; }
  int getTheta() const { return _theta; }

  void setPOS(int x, int y, int theta) {
    _x = x;
    _y = y;
    _theta = theta;
  }

  int get_Wheelbase() const { return _settings.get_Wheelbase(); }
  int get_WheelDiameter() const { return _settings.get_WheelDiameter(); }
  int get_GearRatio() const { return _settings.get_GearRatio(); }

  void set_Wheelbase(int Wheelbase) { _settings.set_Wheelbase(Wheelbase); }
  void set_WheelDiameter(int WheelDiameter) {
    _settings.set_WheelDiameter(WheelDiameter);
  }
  void set_GearRatio(int GearRatio) { _settings.set_GearRatio(GearRatio); }

  void stop() {
    _leftMotors.brake();
    _leftMotors.brake();
  }
};
class Drivetrain4DummiesNT {

public:
  /* ========================= CLASS ========================= */

  Drivetrain4DummiesNT(MotorGroup &_leftMotors, int leftDistance,
                       MotorGroup &_rightMotors, int rightDistance,
                       DTSettingsNT &_settings, Imu &_inertial,
                       Controller &_controller);
  MotorGroup &_leftMotors;
  MotorGroup &_rightMotors;
  Imu &_inertial;
  DTSettingsNT &_settings;
  Controller &_controller;

  // CLAIBRATION

  void CALIBRATE(BrakingType BrakeType = COAST) {
    _inertial.reset();
    while (_inertial.is_calibrating()) {
      pros::delay(20);
    }
  }
  /* =========================================================== */

  /* =========================== GET =========================== */

  int get_wheelbase() const { return _settings.get_Wheelbase(); }
  int get_wheelDiameter() const { return _settings.get_WheelDiameter(); }
  int get_gearRatio() const { return _settings.get_GearRatio(); }
  int get_driveVelocity() const { return _settings.get_driveVelocity(); }
  int get_turnVelocity() const { return _settings.get_turnVelocity(); }
  /* =========================================================== */

  /* =========================== SET =========================== */

  void set_Wheelbase(int Wheelbase) { _settings.set_Wheelbase(Wheelbase); }
  void set_WheelDiameter(int WheelDiameter) {
    _settings.set_WheelDiameter(WheelDiameter);
  }
  void set_GearRatio(int GearRatio) { _settings.set_GearRatio(GearRatio); }
  void set_driveVelocity(int driveVelocity) {
    _settings.set_driveVelocity(driveVelocity);
  }
  void set_turnVelocity(int turnVelocity) {
    _settings.set_turnVelocity(turnVelocity);
  }
  /* =========================================================== */

  /* ========================= MOVEMENT DRIVE ========================= */

  void move_Drive(DirectionStraight direction, uint velocity = 1000,
                  uint timeout = 0) {
    if (velocity == 1000)
      velocity = _settings.get_driveVelocity();
    switch (direction) {
    case 1:
      _leftMotors.move(velocity);
      _rightMotors.move(velocity);
      break;

    case -1:
      _leftMotors.move(-velocity);
      _rightMotors.move(-velocity);
      break;
    }
    delay(timeout);
  }

  void move_DriveFor(DirectionStraight direction, uint distance,
                     uint velocity = 1000, uint timeout = 0,
                     bool async = true) {
    if (velocity == 1000)
      velocity = _settings.get_driveVelocity();
    int start = _leftMotors.get_position();

    int Motordegrees =
        360 * ((distance / (M_PI * _settings.get_WheelDiameter())) *
               _settings.get_GearRatio());

    switch (direction) {
    case 1:
      _leftMotors.move_relative(Motordegrees, velocity);
      _leftMotors.move_relative(Motordegrees, velocity);
      break;

    case -1:
      _leftMotors.move_relative(-Motordegrees, -velocity);
      _leftMotors.move_relative(-Motordegrees, -velocity);
      break;
    }
    if (async) {
      while (_leftMotors.get_position() - start < Motordegrees)
        delay(20);
    }
    delay(timeout);
  }
  /* ================================================================== */

  /* ========================= MOVEMENT TURN ========================= */

  void move_Turn(DirectionTurn direction, uint velocity = 1000,
                 uint timeout = 0) {
    if (velocity == 1000)
      velocity = _settings.get_turnVelocity();
    switch (direction) {
    case 1:
      _leftMotors.move(-velocity);
      _rightMotors.move(velocity);
      break;

    case -1:
      _leftMotors.move(velocity);
      _rightMotors.move(-velocity);
      break;
    }
    delay(timeout);
  }

  void move_TurnFor(DirectionTurn direction, uint theta, uint velocity = 1000,
                    uint timeout = 0, bool async = true) {
    if (velocity == 1000)
      velocity = _settings.get_turnVelocity();
    int Motordegrees =
        (_settings.get_Wheelbase() * theta) / _settings.get_WheelDiameter();
    int start = _leftMotors.get_position();
    switch (direction) {
    case 1:
      _leftMotors.move_relative(-Motordegrees, -velocity);
      _rightMotors.move_relative(Motordegrees, velocity);
      break;

    case -1:
      _leftMotors.move_relative(Motordegrees, velocity);
      _rightMotors.move_relative(-Motordegrees, -velocity);
      break;
    }
    if (async) {
      while (_leftMotors.get_position() - start < Motordegrees)
        delay(20);
    }
    delay(timeout);
  }
  /* ================================================================= */

  /* ========================= MOVEMENT STOP ========================= */

  void stop(BrakingType type = DEFAULT) {
    switch (type) {
    case COAST:
    case DEFAULT:
      _leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      _rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      break;
    case HOLD:
      _leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      break;
    case BRAKE:
      _leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      _rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      break;
    }
    _leftMotors.brake();
    _leftMotors.brake();
  }
  /* ================================================================= */
};
} // namespace v5
} // namespace pros

#endif // PROS_DRIVETRAINS4DUMMIES_HPP