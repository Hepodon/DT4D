// sup this is my files

#ifndef PROS_DRIVETRAINS4DUMMIES_HPP
#define PROS_DRIVETRAINS4DUMMIES_HPP

#include "math.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

namespace pros {
inline namespace v5 {

class DTSettings {

public:
  DTSettings(int Wheelbase, int WheelDiameter, int GearRatio,
             int driveVelocity = 50, int turnVelocity = 35)
      : _wheelbase(Wheelbase), _wheelDiameter(WheelDiameter),
        _gearRatio(GearRatio), _driveVelocity(driveVelocity),
        _turnVelocity(turnVelocity) {}

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

private:
  int _wheelbase;
  int _wheelDiameter;
  int _gearRatio;
  int _driveVelocity;
  int _turnVelocity;
};

class Drivetrain4Dummies {

public:
  Drivetrain4Dummies(MotorGroup &_leftMotors, MotorGroup &_rightMotors,
                     DTSettings &_settings, Imu &_inertial);
  MotorGroup &_leftMotors;
  MotorGroup &_rightMotors;
  Imu &_inertial;
  DTSettings &_settings;

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
  int get_driveVelocity() const { return _settings.get_driveVelocity(); }
  int get_turnVelocity() const { return _settings.get_turnVelocity(); }

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

  void drive(int velocity) {
    _leftMotors.move(velocity);
    _rightMotors.move(velocity);
  }

  void stop() {
    _leftMotors.brake();
    _leftMotors.brake();
  }

private:
};
} // namespace v5
} // namespace pros

#endif // PROS_DRIVETRAINS4DUMMIES_HPP