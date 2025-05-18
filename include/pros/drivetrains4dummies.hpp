// sup this is my files

#pragma once

#ifndef PROS_DRIVETRAINS4DUMMIES_HPP
#define PROS_DRIVETRAINS4DUMMIES_HPP

#include "math.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

enum DirectionStraight { Forward = 1, Fwd = 1, Reverse = -1, Rev = -1 };
enum DirectionTurn { Left = 1, L = 1, Right = -1, R = -1 };
enum BrakingType { COAST = 0, BRAKE = 1, HOLD = 2, DEFAULT = 3 };
enum MoveType { PID, Relative, Basic };

namespace pros {
inline namespace v5 {
namespace DT4D {
float applySlew(int current, int target, float rate = 5);

/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
class PIDsettings {
public:
  PIDsettings(int checkTime, double threshold, double minSpeed,
              double maxSpeed);

  int get_checkTime() const;
  double get_threshold() const;
  double get_minSpeed() const;
  double get_maxSpeed() const;

private:
  int _checkTime;
  double _threshold;
  double _minSpeed;
  double _maxSpeed;
};
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
class DTSettingsNT {

public:
  DTSettingsNT(int Wheelbase, int WheelDiameter, float GearRatio, double kP,
               double kI, double kD, double DkP, double DkI, double DkD,
               int driveVelocity = 50, int turnVelocity = 35)
      : _wheelbase(Wheelbase), _wheelDiameter(WheelDiameter),
        _gearRatio(GearRatio), _driveVelocity(driveVelocity),
        _turnVelocity(turnVelocity), _kP(kP), _kI(kI), _kD(kD), D_kP(DkP),
        D_kI(DkI), D_kD(DkD) {}

  int get_Wheelbase() const;
  int get_WheelDiameter() const;
  float get_GearRatio() const;
  int get_driveVelocity() const;
  int get_turnVelocity() const;

  void set_Wheelbase(int Wheelbase);
  void set_WheelDiameter(int WheelDiameter);
  void set_GearRatio(int GearRatio);
  void set_driveVelocity(int driveVelocity);
  void set_turnVelocity(int turnVelocity);

  double get_kP() const;
  double get_kI() const;
  double get_kD() const;
  double get_DkP() const;
  double get_DkI() const;
  double get_DkD() const;

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
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
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

/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/

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

  void CALIBRATE(int x = 0, int y = 0, int theta = 0);

  int get_wheelbase() const;
  int get_wheelDiameter() const;
  int get_gearRatio() const;

  void set_Wheelbase(int Wheelbase);
  void set_WheelDiameter(int WheelDiameter);
  void set_GearRatio(int GearRatio);
  void set_Pos(int x, int y);
  void set_angle(int theta);

  void moveX(int distance);

  void moveY(int distance);

  void move_ToPos(int x, int y, bool Xfirst = true);

  void move_ToPosDiag(int x, int y);

private:
  void move_DriveFor(DirectionStraight direction, uint distance,
                     int slewRate = 15, uint velocity = 1000, uint timeout = 0,
                     bool async = true);

  void move_TurnFor(DirectionTurn direction, uint theta, int slewRate = 5,
                    uint timeout = 0, bool async = true);

  void turn_ToAngle(int angle, DirectionTurn Direction);
};
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
/*==========================================================================*/
class Drivetrain4DummiesNT {

public:
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

  void CALIBRATE();

  int get_wheelbase() const;
  int get_wheelDiameter() const;
  int get_gearRatio() const;
  int get_driveVelocity() const;
  int get_turnVelocity() const;

  void set_Wheelbase(int Wheelbase);
  void set_WheelDiameter(int WheelDiameter);
  void set_GearRatio(int GearRatio);
  void set_driveVelocity(int driveVelocity);
  void set_turnVelocity(int turnVelocity);

  void move_Drive(DirectionStraight direction, uint velocity = 1000,
                  uint timeout = 0);

  void move_DriveFor(MoveType MT, DirectionStraight direction, uint distance,
                     int slewRate = 15, uint velocity = 1000, uint timeout = 0,
                     bool async = true);

  void move_Turn(DirectionTurn direction, uint velocity = 1000,
                 uint timeout = 0);

  void move_TurnFor(MoveType MT, DirectionTurn direction, uint theta,
                    int slewRate = 5, uint velocity = 1000, uint timeout = 0,
                    bool async = true);

  void stop(BrakingType type = DEFAULT);
};
} // namespace DT4D
} // namespace v5
} // namespace pros

#endif // PROS_DRIVETRAINS4DUMMIES_HPP