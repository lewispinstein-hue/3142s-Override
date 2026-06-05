#include "drive.hpp"
#include "setup.hpp"
#include <algorithm>
#include <cmath>

namespace control {
Velocities arcade(double throttle, double turn, float desaturateBias) {
  // desaturate motors based on joyBias
  if (std::abs(throttle) + std::abs(turn) > 127) {
    const double oldThrottle = throttle;
    const double oldTurn = turn;
    throttle *= (1 - desaturateBias * std::abs(oldTurn / 127.0));
    turn *= (1 - (1 - desaturateBias) * std::abs(oldThrottle / 127.0));
  }

  float leftPower = throttle + turn;
  float rightPower = throttle - turn;

  return {leftPower, rightPower};
}

Velocities curvature(double throttle, double turn) {
  // If we're not moving forwards change to arcade drive
  if (throttle == 0) {
    return arcade(throttle, turn, 0.5);
  }

  // use drive curves if they have not been disabled
  float leftPower = throttle + (std::fabs(throttle) * turn / 127.0);
  float rightPower = throttle - (std::fabs(throttle) * turn / 127.0);

  // desaturate output
  float max = std::max(std::fabs(leftPower), std::fabs(rightPower)) / 127;
  if (max > 1) {
    leftPower /= max;
    rightPower /= max;
  }
  return {leftPower, rightPower};
}

double normVel(double rpm) {
  double v = (rpm / 4.7244094488); // Locked at blue gearset
  // If your using a different gearset, change this to (maxRPM / 127)
  return std::clamp(v, -127.0, 127.0);
}

double slewLimit(double target, float& prev, MotionType type, const Slew& slew) {

  if (target == prev) return target;
  double delta = target - prev;
  const bool sameDirection = (prev * target > 0);

  double riseDelta = (type == MotionType::FORWARD) ? slew.risingThrottle : slew.risingAngle;
  double fallDelta = (type == MotionType::FORWARD) ? slew.fallingThrottle : slew.fallingAngle;

  // Starting from zero is still acceleration; direction reversals use the falling limit.
  bool accelerating = (std::fabs(prev) == 0 || sameDirection) && (std::fabs(target) > std::fabs(prev));
  double maxDelta = accelerating ? riseDelta : fallDelta;
  if (delta > maxDelta) delta = maxDelta;
  if (delta < -maxDelta) delta = -maxDelta;
  prev += delta;
  return prev;
}

double expoThrottle(double input, double expoThrottle, double deadband) {
  if (fabs(input) < deadband) return 0;
  double norm = input / 127.0;
  double curved = pow(fabs(norm), expoThrottle);
  curved = curved * (norm >= 0 ? 1 : -1); 
  if (fabs(curved) >= 1) return 127 * ((norm >= 0) ? 1 : -1);
  return curved * 127.0;
}

double expoTurn(double input, const ExpoTurnConfig& config) {
  const bool controlOverride =
    std::abs(controller.get_analog(ANALOG_RIGHT_X)) > config.joystickSpeedOverrideThreshold;
  
  const double speed = std::fabs(normVel(
    (leftDrivetrain.get_actual_velocity() + rightDrivetrain.get_actual_velocity()) / 2
  ));

  const double turnMultiplier =
    (speed > config.robotSpeedOverrideThreshold || controlOverride)
      ? config.overrideSpeedMultiplier
      : config.defaultSpeedMultiplier;

  double norm = input / 127.0;
  double linear = norm;
  double exponential = pow(fabs(norm), config.expoTurn) * ((norm >= 0) ? 1 : -1);
  double blended = (0.39 * linear + 0.42 * exponential) / (0.39 + 0.42);
  if (fabs(blended) >= 1) return 127 * ((norm >= 0) ? 1 : -1);
  return blended * turnMultiplier;
}

void updateDrive(const DriveConfig& config) {
  int throttle = controller.get_analog(ANALOG_LEFT_Y);
  int turn = controller.get_analog(ANALOG_RIGHT_X);

  if (abs(throttle) < config.deadband) throttle = 0;
  if (abs(turn) < config.deadband) turn = 0;

  static float prevThrottle = 0;
  static float prevTurn = 0;

  const float processedThrottle = slewLimit(
    expoThrottle(throttle, config.expoThrottle, config.deadband),
    prevThrottle,
    MotionType::FORWARD, 
    config.slew
  );

  const float processedTurn = slewLimit(
    expoTurn(turn, config.expoTurnConfig),
    prevTurn,
    MotionType::TURN, 
    config.slew
  );

  Velocities velocities;
  if (config.driveMode == DriveMode::ARCADE) {
    velocities = arcade(processedThrottle, processedTurn, config.desaturateBias);
  } else {
    velocities = curvature(processedThrottle, processedTurn);
  }

  leftDrivetrain.move(velocities.leftVelocity);
  rightDrivetrain.move(velocities.rightVelocity);
}
} // namespace control
