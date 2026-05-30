#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "setup.hpp"

namespace control {
/**
 * @struct Slew
 * @brief Slew rate limiter. Used with the slewLimit function
 * @note Set values to 0 to disable slew on that axis
*/
struct Slew {
  /** @brief Slew applied when increasing magnitude/speed (in turning mode) */
  float risingAngle; 

  /** @brief Slew applied when decreasing magnitude/speed (in turning mode) */
  float fallingAngle;

  // |---|<*>|---| //

  /** @brief Slew applied when increasing magnitude/speed (in forward mode) */
  float risingThrottle;

  /** @brief Slew applied when decreasing magnitude/speed (in forward mode) */
  float fallingThrottle;

  static constexpr float infinity = std::numeric_limits<float>::infinity();
  Slew(float risingAngle, float fallingAngle,
       float risingThrottle, float fallingThrottle)
      : risingAngle(risingAngle <= 0 ? infinity : risingAngle),
        fallingAngle(fallingAngle <= 0 ? infinity : fallingAngle),
        risingThrottle(risingThrottle <= 0 ? infinity : risingThrottle),
        fallingThrottle(fallingThrottle <= 0 ? infinity : fallingThrottle) {}
};

enum class MotionType {
  FORWARD, /// Forward motion, lateral movement
  TURN     /// Turn motion, angular movement
};

enum class DriveMode {
  ARCADE,
  CURVATURE
};

struct ExpoTurnConfig {
  float expoTurn = 1.9;
  int deadband = 20;
  
  int joystickSpeedOverrideThreshold = 126;
  int robotSpeedOverrideThreshold = 100;
  
  float defaultSpeedMultiplier = 90;
  float overrideSpeedMultiplier = 120;
  
  ExpoTurnConfig(float expoTurn, int deadband,
    int joystickSpeedOverrideThreshold,
    int robotSpeedOverrideThreshold, float defaultSpeedMultiplier,
    float overrideSpeedMultiplier)
    : expoTurn(expoTurn), deadband(deadband),
    joystickSpeedOverrideThreshold(joystickSpeedOverrideThreshold),
    robotSpeedOverrideThreshold(robotSpeedOverrideThreshold),
    defaultSpeedMultiplier(defaultSpeedMultiplier),
    overrideSpeedMultiplier(overrideSpeedMultiplier) {}
};

struct DriveConfig {
  Slew slew;
  ExpoTurnConfig expoTurnConfig;
  DriveMode driveMode;
  float expoThrottle;
  float expoTurn;
  float deadband;
  float desaturateBias;
};

/**
 * @brief Slew rate limiter
 * 
 * @param target target value
 * @param prev previous value
 * @param type forward or turn
 * @param slew slew struct
 * 
 * @return Limited @c target value with applied slew based on type, slew, and prev
*/
double slewLimit(double target, float& prev, MotionType type, const Slew& slew);

double expoTurn(double input, const ExpoTurnConfig& config);
double expoThrottle(double input, double expoThrottle, double deadband);

struct Velocities {
  float leftVelocity;
  float rightVelocity;
};

Velocities arcade(int throttle, int turn, float desaturateBias);
Velocities curvature(int throttle, int turn);

void updateDrive(const DriveConfig& config);

} // namespace control
