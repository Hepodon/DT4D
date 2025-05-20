#include "pros/drivetrains4dummies.hpp"

namespace pros {
namespace DT4D {

float applySlew(int current, int target, float rate) {
  int diff = target - current;
  if (abs(diff) > rate)
    return current + rate * (diff > 0 ? 1 : -1);
  return target;
}

int PIDsettings::get_checkTime() const { return _checkTime; }

int PIDsettings::get_threshold() const { return _threshold; }

int PIDsettings::get_minSpeed() const { return _minSpeed; }

int PIDsettings::get_maxSpeed() const { return _maxSpeed; }

int DTSettingsNT::get_Wheelbase() const { return _wheelbase; }
int DTSettingsNT::get_WheelDiameter() const { return _wheelDiameter; }
float DTSettingsNT::get_GearRatio() const { return _gearRatio; }
int DTSettingsNT::get_driveVelocity() const { return _driveVelocity; }
int DTSettingsNT::get_turnVelocity() const { return _turnVelocity; }

void DTSettingsNT::set_Wheelbase(int Wheelbase) { _wheelbase = Wheelbase; }
void DTSettingsNT::set_WheelDiameter(int WheelDiameter) {
  _wheelDiameter = WheelDiameter;
}
void DTSettingsNT::set_gearRatio(float GearRatio) { _gearRatio = GearRatio; }
void DTSettingsNT::set_driveVelocity(int driveVelocity) {
  _driveVelocity = driveVelocity;
}
void DTSettingsNT::set_turnVelocity(int turnVelocity) {
  _turnVelocity = turnVelocity;
}

double DTSettingsNT::get_kP() const { return _kP; }
double DTSettingsNT::get_kI() const { return _kI; }
double DTSettingsNT::get_kD() const { return _kD; }

double DTSettingsNT::get_DkP() const { return D_kP; }
double DTSettingsNT::get_DkI() const { return D_kI; }
double DTSettingsNT::get_DkD() const { return D_kD; }

void Drivetrain4Dummies::CALIBRATE(int x, int y, int theta) {
  _inertial.reset(true);
  _x = x;
  _y = y;
  _theta = theta;
}

int Drivetrain4Dummies::get_wheelbase() const {
  return _settings.get_Wheelbase();
}
int Drivetrain4Dummies::get_wheelDiameter() const {
  return _settings.get_WheelDiameter();
}
int Drivetrain4Dummies::get_gearRatio() const {
  return _settings.get_GearRatio();
}

void Drivetrain4Dummies::set_Wheelbase(int Wheelbase) {
  _settings.set_Wheelbase(Wheelbase);
}
void Drivetrain4Dummies::set_WheelDiameter(int WheelDiameter) {
  _settings.set_WheelDiameter(WheelDiameter);
}
void Drivetrain4Dummies::set_gearRatio(float GearRatio) {
  _settings.set_gearRatio(GearRatio);
}
void Drivetrain4Dummies::set_Pos(int x, int y) {
  _x = x;
  _y = y;
}
void Drivetrain4Dummies::set_angle(int theta) { _theta = theta; }

void Drivetrain4Dummies::moveX(int distance) {
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
  Drivetrain4Dummies::move_DriveFor(Fwd, distance);
  _x += distance;
}

void Drivetrain4Dummies::moveY(int distance) {
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
  _y = distance;
}

void Drivetrain4Dummies::move_ToPos(int x, int y, bool Xfirst) {
  int deltaX = x - _x;
  int deltaY = y - _y;
  if (Xfirst) {
    moveX(deltaX);
    moveY(deltaY);
  } else {
    moveY(deltaY);
    moveX(deltaX);
  }
  _x = x;
  _y = y;
}

void Drivetrain4Dummies::move_ToPosDiag(int x, int y) {
  int deltaX = x - _x;
  int deltaY = y - _x;

  float distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  float theta = tan(deltaY / deltaX);

  turn_ToAngle(theta, L);
  move_DriveFor(Fwd, distance);

  _x = x;
  _y = y;
}

void Drivetrain4Dummies::move_DriveFor(DirectionStraight direction,
                                       uint distance, int slewRate,
                                       uint velocity, uint timeout,
                                       bool async) {
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

    output = std::clamp(output, static_cast<double>(_PIDset.get_minSpeed()),
                        static_cast<double>(_PIDset.get_maxSpeed()));

    double delta = output - lastOutput;
    if (std::abs(delta) > slewRate)
      output = lastOutput + slewRate * (delta > 0 ? 1 : -1);

    lastOutput = output;

    _leftMotors.move_velocity(output);
    _rightMotors.move_velocity(output);
    delay(20);
  }
}

void Drivetrain4Dummies::move_TurnFor(DirectionTurn direction, uint theta,
                                      int slewRate, uint timeout, bool async) {
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

void Drivetrain4Dummies::turn_ToAngle(int angle, DirectionTurn Direction) {
  int deltaT = angle - _theta;
  move_TurnFor(Direction, deltaT);
}

void Drivetrain4DummiesNT::CALIBRATE() { _inertial.reset(true); }
/* =========================================================== */

/* =========================== GET =========================== */

int Drivetrain4DummiesNT::get_wheelbase() const {
  return _settings.get_Wheelbase();
}
int Drivetrain4DummiesNT::get_wheelDiameter() const {
  return _settings.get_WheelDiameter();
}
int Drivetrain4DummiesNT::get_gearRatio() const {
  return _settings.get_GearRatio();
}
int Drivetrain4DummiesNT::get_driveVelocity() const {
  return _settings.get_driveVelocity();
}
int Drivetrain4DummiesNT::get_turnVelocity() const {
  return _settings.get_turnVelocity();
}
/* =========================================================== */

/* =========================== SET =========================== */

void Drivetrain4DummiesNT::set_Wheelbase(int Wheelbase) {
  _settings.set_Wheelbase(Wheelbase);
}
void Drivetrain4DummiesNT::set_WheelDiameter(int WheelDiameter) {
  _settings.set_WheelDiameter(WheelDiameter);
}
void Drivetrain4DummiesNT::set_gearRatio(float gearRatio) {
  _settings.set_gearRatio(gearRatio);
}
void Drivetrain4DummiesNT::set_driveVelocity(int driveVelocity) {
  _settings.set_driveVelocity(driveVelocity);
}
void Drivetrain4DummiesNT::set_turnVelocity(int turnVelocity) {
  _settings.set_turnVelocity(turnVelocity);
}
/* =========================================================== */

/* ========================= MOVEMENT DRIVE ========================= */

void Drivetrain4DummiesNT::move_Drive(DirectionStraight direction,
                                      uint velocity, uint timeout) {
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

void Drivetrain4DummiesNT::move_DriveFor(MoveType MT,
                                         DirectionStraight direction,
                                         uint distance, int slewRate,
                                         uint velocity, uint timeout,
                                         bool async) {
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

      output = std::clamp(output, static_cast<double>(_PIDset.get_minSpeed()),
                          static_cast<double>(_PIDset.get_maxSpeed()));
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

void Drivetrain4DummiesNT::move_Turn(DirectionTurn direction, uint velocity,
                                     uint timeout) {
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

void Drivetrain4DummiesNT::move_TurnFor(MoveType MT, DirectionTurn direction,
                                        uint theta, int slewRate, uint velocity,
                                        uint timeout, bool async) {
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

void Drivetrain4DummiesNT::stop(BrakingType type) {
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
/* ================================================================= */ // namespace
                                                                        // DT4D
} // namespace DT4D
} // namespace pros