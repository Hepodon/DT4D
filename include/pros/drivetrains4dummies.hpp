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
namespace DT4D {

/**
 *  Applys slew rates to value input with
 * current value and target, using optional rate
 *
 *@param current int Current value
 *@param target int target value
 *@param rate float rate of change value
 *
 */
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
  /**
   * Constructs a PIDsettings object with custom PID parameters.
   *
   * @param checkTime   int Time to remain within threshold to consider settled.
   * @param threshold   double Error threshold to stop movement.
   * @param minSpeed    double Minimum motor speed.
   * @param maxSpeed    double Maximum motor speed.
   */
  PIDsettings(int checkTime, int threshold, int minSpeed, int maxSpeed)
      : _checkTime(checkTime), _threshold(threshold), _minSpeed(minSpeed),
        _maxSpeed(maxSpeed) {}

  /// Returns int checkTime as a constant
  int get_checkTime() const;

  /// Returns double threshold as a constant
  int get_threshold() const;

  /// Returns double minSpeed as a constant
  int get_minSpeed() const;

  /// Returns double maxSpeed as a constant
  int get_maxSpeed() const;

private:
  int _checkTime;
  int _threshold;
  int _minSpeed;
  int _maxSpeed;
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
  /**
   * Constructs a DTSettingsNT object with custom drivetrain parameters
   * using chassis measurements and previous constructs
   *This is used for No Tracking Drivetrain settings
   *
   *@param Wheelbase int distance in inches between front left and right wheels
   *@param WheelDiameter int diameter of wheel in inches
   *@param gearRatio float gear ratio of motor to wheels
   *@param kp double turning proportional gain
   *@param ki double turning integral gain
   *@param kd doube turning derivative gain
   *@param DkP double driving proportional gain
   *@param DkI double driving integral gain
   *@param DkD double driving derivative gain
   */
  DTSettingsNT(int Wheelbase, int WheelDiameter, float GearRatio, double kP,
               double kI, double kD, double DkP, double DkI, double DkD,
               int driveVelocity = 50, int turnVelocity = 35)
      : _wheelbase(Wheelbase), _wheelDiameter(WheelDiameter),
        _gearRatio(GearRatio), _driveVelocity(driveVelocity),
        _turnVelocity(turnVelocity), _kP(kP), _kI(kI), _kD(kD), D_kP(DkP),
        D_kI(DkI), D_kD(DkD) {}

  /// Returns int wheelbase as a constant
  int get_Wheelbase() const;

  /// Returns int wheelDiameter as a constant
  int get_WheelDiameter() const;

  /// Returns int gearRatio as a constant
  float get_GearRatio() const;

  /// Returns int drive velocity as a constant
  int get_driveVelocity() const;

  /// Returns int turn velocity as a constant
  int get_turnVelocity() const;

  /**
   * provides new value for wheelbase construct
   *@param wheelbase int distance in inches between front left and right wheels
   */
  void set_Wheelbase(int Wheelbase);

  /**
   * provides new value for wheelDiameter construct
   *@param wheelDiameter int diameter of wheel in inches
   */
  void set_WheelDiameter(int WheelDiameter);

  /**
   * provides new value for GearRatio construct
   *@param gearRatio int distance in inches between front left and right wheels
   */
  void set_gearRatio(float gearRatio);

  /**
   * provides new value for diving velocity construct
   *@param driveVelocity int defualt velocity used for driving
   */
  void set_driveVelocity(int driveVelocity);

  /**
   * provides new value for turning velocity construct
   *@param turnVelocity int defualt velocity used for turning
   */
  void set_turnVelocity(int turnVelocity);

  /// Returns int turning Proportional as a constant
  double get_kP() const;

  /// Returns int turning Integral as a constant
  double get_kI() const;

  /// Returns int turning Derivative as a constant
  double get_kD() const;

  /// Returns int driving Proportional as a constant
  double get_DkP() const;

  /// Returns int driving Integral as a constant
  double get_DkI() const;

  /// Returns int driving Derivative as a constant
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

  int get_Wheelbase() const;
  int get_WheelDiameter() const;
  float get_GearRatio() const;

  void set_Wheelbase(int Wheelbase);
  void set_WheelDiameter(int WheelDiameter);
  void set_gearRatio(int gearRatio);

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
  void set_gearRatio(float gearRatio);
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
  void set_gearRatio(float gearRatio);
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
} // namespace pros

#endif // PROS_DRIVETRAINS4DUMMIES_HPP