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
class PIDsettings {
public:
  PIDsettings(int checkTime, double threshold, double minSpeed, double maxSpeed)
      : _maxSpeed(maxSpeed), _minSpeed(minSpeed), _threshold(threshold),
        _checkTime(checkTime) {}

  int get_checkTime() const { return _checkTime; };
  double get_threshold() const { return _threshold; };
  double get_minSpeed() const { return _minSpeed; };
  double get_maxSpeed() const { return _maxSpeed; };

  int _checkTime;
  double _threshold;
  double _minSpeed;
  double _maxSpeed;
};
class DTSettingsNT {

public:
  DTSettingsNT(int Wheelbase, int WheelDiameter, float GearRatio, double kP,
               double kI, double kD, double DkP, double DkI, double DkD,
               int driveVelocity = 50, int turnVelocity = 35)
      : _wheelbase(Wheelbase), _wheelDiameter(WheelDiameter),
        _gearRatio(GearRatio), _driveVelocity(driveVelocity),
        _turnVelocity(turnVelocity), _kP(kP), _kI(kI), _kD(kD), D_kP(DkP),
        D_kI(DkI), D_kD(DkD) {}

  int get_Wheelbase() const { return _wheelbase; }
  int get_WheelDiameter() const { return _wheelDiameter; }
  float get_GearRatio() const { return _gearRatio; }
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

  double _kP;
  double _kI;
  double _kD;

  double D_kP;
  double D_kI;
  double D_kD;

  int _wheelbase;
  int _wheelDiameter;
  float _gearRatio;
  int _driveVelocity;
  int _turnVelocity;
};
class DTSettings {

public:
  DTSettings(int Wheelbase, int WheelDiameter, float GearRatio, double kP,
             double kI, double kD, double DkP, double DkI, double DkD)
      : _wheelbase(Wheelbase), _wheelDiameter(WheelDiameter),
        _gearRatio(GearRatio), _kP(kP), _kI(kI), _kD(kD), D_kP(DkP), D_kI(DkI),
        D_kD(DkD) {}

  int get_Wheelbase() const { return _wheelbase; }
  int get_WheelDiameter() const { return _wheelDiameter; }
  float get_GearRatio() const { return _gearRatio; }

  void set_Wheelbase(int Wheelbase) { _wheelbase = Wheelbase; }
  void set_WheelDiameter(int WheelDiameter) { _wheelDiameter = WheelDiameter; }
  void set_GearRatio(int GearRatio) { _gearRatio = GearRatio; }

  double get_kP() const { return _kP; }
  double get_kI() const { return _kI; }
  double get_kD() const { return _kD; }

  double get_DkP() const { return D_kP; }
  double get_DkI() const { return D_kI; }
  double get_DkD() const { return D_kD; }

  double _kP;
  double _kI;
  double _kD;

  double D_kP;
  double D_kI;
  double D_kD;

  int _wheelbase;
  int _wheelDiameter;
  int _gearRatio;
};
class Drivetrain4Dummies {

public:
  Drivetrain4Dummies(PIDsettings &PIDset, MotorGroup &leftMotors,
                     int leftDistance, MotorGroup &rightMotors,
                     int rightDistance, DTSettings &settings, Imu &inertial,
                     Controller *con = nullptr)
      : _leftMotors(leftMotors), _rightMotors(rightMotors), _inertial(inertial),
        _settings(settings), _con(con), _PIDset(PIDset) {}
  MotorGroup &_leftMotors;
  MotorGroup &_rightMotors;
  Imu &_inertial;
  DTSettings &_settings;
  Controller *_con;
  PIDsettings &_PIDset;

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
    if (_con != nullptr)
      _con->rumble(".-.");
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

  Drivetrain4DummiesNT(PIDsettings &PIDset, MotorGroup &leftMotors,
                       int leftDistance, MotorGroup &rightMotors,
                       int rightDistance, DTSettingsNT &settings, Imu &inertial,
                       Controller &controller)
      : _leftMotors(leftMotors), _rightMotors(rightMotors), _inertial(inertial),
        _settings(settings), _controller(controller), _PIDset(PIDset) {}
  MotorGroup &_leftMotors;
  MotorGroup &_rightMotors;
  Imu &_inertial;
  DTSettingsNT &_settings;
  Controller &_controller;
  PIDsettings &_PIDset;

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

  void move_DriveFor(bool PID, DirectionStraight direction, uint distance,
                     uint velocity = 1000, uint timeout = 0,
                     bool async = true) {
    if (!PID) {
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
    } else {
      _leftMotors.tare_position();
      _leftMotors.tare_position();

      double error;
      double lastError = 0;
      double derivative;
      double integral = 0;

      const double wheelCircumference = _settings.get_WheelDiameter() * M_PI;
      const double degPerInch =
          360.0 * _settings.get_GearRatio() / wheelCircumference;
      const double targetPosition = distance * degPerInch;

      while (true) {
        int currentPosition =
            (_leftMotors.get_position() + -_rightMotors.get_position()) / 2.0;
        error = targetPosition - currentPosition;

        if (fabs(error) < _PIDset.get_threshold())
          break;

        integral += error;
        derivative = error - lastError;
        lastError = error;

        double output = _settings.get_DkP() * error +
                        _settings.get_DkI() * integral +
                        _settings.get_DkD() * derivative;

        output = std::max(std::min(output, _PIDset.get_maxSpeed()),
                          _PIDset.get_minSpeed());

        _leftMotors.move_velocity(output);
        _rightMotors.move_velocity(output);
        delay(20);
      }
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

  void move_TurnFor(bool PID, DirectionTurn direction, uint theta,
                    uint velocity = 1000, uint timeout = 0, bool async = true) {
    if (!PID) {
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
    } else {
      double error = 0;
      double lastError = 0;
      double integral = 0;
      double derivative = 0;
      double output = 0;

      const double maxOutput = _PIDset.get_maxSpeed();
      const double integralLimit = maxOutput;
      const double minSpeed = _PIDset.get_minSpeed();
      const double threshold = _PIDset.get_threshold();
      const int settleTime = _PIDset.get_checkTime();

      int withinThresholdTime = 0;

      while (withinThresholdTime < settleTime) {
        double currentAngle = _inertial.get_heading();
        error = theta - currentAngle;

        integral += error;
        if (std::abs(error) > integralLimit)
          integral = 0;

        derivative = error - lastError;
        lastError = error;

        output = _settings.get_kP() * error + _settings.get_kI() * integral +
                 _settings.get_kD() * derivative;

        output = std::clamp(output, -maxOutput, maxOutput);

        if (std::abs(output) < minSpeed && std::abs(error) > threshold)
          output = minSpeed * (output > 0 ? 1 : -1);

        _leftMotors.move(-output);
        _rightMotors.move(output);

        if (std::abs(error) < threshold)
          withinThresholdTime += 10;
        else
          withinThresholdTime = 0;

        pros::delay(10);
      }
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