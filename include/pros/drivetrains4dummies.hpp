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
enum MoveType { PID, Relative, Basic };
/* ========================================================== */

namespace pros {
inline namespace v5 {
namespace DT4D {
inline float applySlew(int current, int target, float rate = 5) {
  int diff = target - current;
  if (abs(diff) > rate)
    return current + rate * (diff > 0 ? 1 : -1);
  return target;
}

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

  int _theta;
  int _x;
  int _y;

  void CALIBRATE(int x = 0, int y = 0, int theta = 0) {
    _inertial.reset(true);
    _x = x;
    _y = y;
    _theta = theta;
  }

  int get_wheelbase() const { return _settings.get_Wheelbase(); }
  int get_wheelDiameter() const { return _settings.get_WheelDiameter(); }
  int get_gearRatio() const { return _settings.get_GearRatio(); }

  void set_Wheelbase(int Wheelbase) { _settings.set_Wheelbase(Wheelbase); }
  void set_WheelDiameter(int WheelDiameter) {
    _settings.set_WheelDiameter(WheelDiameter);
  }
  void set_GearRatio(int GearRatio) { _settings.set_GearRatio(GearRatio); }
  void set_Pos(int x, int y) {
    _x = x;
    _y = y;
  }
  void set_angle(int theta) { _theta = theta; }

  void moveX(int distance) {
    if (distance > 0) {
      if (_theta < 90) {
        turn_ToAngle(90, R);
      } else {
        turn_ToAngle(90, L);
      }
    } else {
      if (_theta < -90) {
        turn_ToAngle(90, R);
      } else {
        turn_ToAngle(90, L);
      }
    }
    move_DriveFor(Fwd, distance);
  }

  void moveY(int distance) {
    if (distance > 0) {
      if (_theta < 0) {
        turn_ToAngle(0, R);
      } else {
        turn_ToAngle(0, L);
      }
    } else {
      if (_theta > 0) {
        if (_theta < 180) {
          turn_ToAngle(90, R);
        } else {
          turn_ToAngle(90, L);
        }
      } else {
        if (_theta < -180) {
          turn_ToAngle(90, R);
        } else {
          turn_ToAngle(90, L);
        }
      }
      move_DriveFor(Fwd, distance);
    }
  }

  void move_ToPos(int x, int y, bool Xfirst = true) {
    int deltaX = x - _x;
    int deltaY = y - _y;
    if (Xfirst) {
      moveX(deltaX);
      moveY(deltaY);
    } else {
      moveY(deltaY);
      moveX(deltaX);
      
    }
  }

private:
  void move_DriveFor(DirectionStraight direction, uint distance,
                     int slewRate = 15, uint velocity = 1000, uint timeout = 0,
                     bool async = true) {
    _leftMotors.tare_position();
    _leftMotors.tare_position();

    double error;
    double lastError = 0;
    double derivative;
    double integral = 0;
    float lastOutput = 0;

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
      double delta = output - lastOutput;
      if (std::abs(delta) > slewRate)
        output = lastOutput + slewRate * (delta > 0 ? 1 : -1);

      lastOutput = output;

      _leftMotors.move_velocity(output);
      _rightMotors.move_velocity(output);
      delay(20);
    }
  }

  void move_TurnFor(DirectionTurn direction, uint theta, int slewRate = 5,
                    uint timeout = 0, bool async = true) {
    _theta = theta;

    double error = 0;
    double lastError = 0;
    double integral = 0;
    double derivative = 0;

    const double maxOutput = _PIDset.get_maxSpeed();
    const double integralLimit = maxOutput;
    const double minSpeed = _PIDset.get_minSpeed();
    const double threshold = _PIDset.get_threshold();
    const int settleTime = _PIDset.get_checkTime();
    double rawOutput = 0;

    float lastOutput = 0;

    int withinThresholdTime = 0;

    while (withinThresholdTime < settleTime) {
      double currentAngle = _inertial.get_heading();
      error = theta - currentAngle;

      integral += error;
      if (std::abs(error) > integralLimit)
        integral = 0;

      derivative = error - lastError;
      lastError = error;

      rawOutput = _settings.get_kP() * error + _settings.get_kI() * integral +
                  _settings.get_kD() * derivative;

      rawOutput = std::clamp(rawOutput, -maxOutput, maxOutput);

      if (std::abs(rawOutput) < minSpeed && std::abs(error) > threshold)
        rawOutput = minSpeed * (rawOutput > 0 ? 1 : -1);

      double delta = rawOutput - lastOutput;
      if (std::abs(delta) > slewRate)
        rawOutput = lastOutput + slewRate * (delta > 0 ? 1 : -1);

      lastOutput = rawOutput;

      _leftMotors.move(-rawOutput);
      _rightMotors.move(rawOutput);

      if (std::abs(error) < threshold)
        withinThresholdTime += 10;
      else
        withinThresholdTime = 0;

      pros::delay(10);
    }
  }

  void turn_ToAngle(int angle, DirectionTurn Direction) {
    int deltaT = angle - _theta;
    move_TurnFor(Direction, deltaT);
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

  void CALIBRATE() { _inertial.reset(true); }
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

  void move_DriveFor(MoveType MT, DirectionStraight direction, uint distance,
                     int slewRate = 15, uint velocity = 1000, uint timeout = 0,
                     bool async = true) {
    switch (MT) {
    case Relative: {
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
      break;
    }
    case PID: {
      _leftMotors.tare_position();
      _leftMotors.tare_position();

      double error;
      double lastError = 0;
      double derivative;
      double integral = 0;
      float lastOutput = 0;

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
        double delta = output - lastOutput;
        if (std::abs(delta) > slewRate)
          output = lastOutput + slewRate * (delta > 0 ? 1 : -1);

        lastOutput = output;

        _leftMotors.move_velocity(output);
        _rightMotors.move_velocity(output);
        delay(20);
      }
    }
    case Basic: {
    }
    }
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

  void move_TurnFor(MoveType MT, DirectionTurn direction, uint theta,
                    int slewRate = 5, uint velocity = 1000, uint timeout = 0,
                    bool async = true) {
    switch (MT) {
    case 1: {
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
      break;
    }
    case 0: {
      double error = 0;
      double lastError = 0;
      double integral = 0;
      double derivative = 0;

      const double maxOutput = _PIDset.get_maxSpeed();
      const double integralLimit = maxOutput;
      const double minSpeed = _PIDset.get_minSpeed();
      const double threshold = _PIDset.get_threshold();
      const int settleTime = _PIDset.get_checkTime();
      double rawOutput = 0;

      float lastOutput = 0;

      int withinThresholdTime = 0;

      while (withinThresholdTime < settleTime) {
        double currentAngle = _inertial.get_heading();
        error = theta - currentAngle;

        integral += error;
        if (std::abs(error) > integralLimit)
          integral = 0;

        derivative = error - lastError;
        lastError = error;

        rawOutput = _settings.get_kP() * error + _settings.get_kI() * integral +
                    _settings.get_kD() * derivative;

        rawOutput = std::clamp(rawOutput, -maxOutput, maxOutput);

        if (std::abs(rawOutput) < minSpeed && std::abs(error) > threshold)
          rawOutput = minSpeed * (rawOutput > 0 ? 1 : -1);

        double delta = rawOutput - lastOutput;
        if (std::abs(delta) > slewRate)
          rawOutput = lastOutput + slewRate * (delta > 0 ? 1 : -1);

        lastOutput = rawOutput;

        _leftMotors.move(-rawOutput);
        _rightMotors.move(rawOutput);

        if (std::abs(error) < threshold)
          withinThresholdTime += 10;
        else
          withinThresholdTime = 0;

        pros::delay(10);
      }
      break;
    }
    case Basic: {
    }
    }
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
} // namespace DT4D
} // namespace v5
} // namespace pros

#endif // PROS_DRIVETRAINS4DUMMIES_HPP