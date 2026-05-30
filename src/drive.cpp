#include "drive.hpp"
#include "setup.hpp"
#include <cmath>

namespace control {
Velocities arcade(int throttle, int turn, float desaturateBias) {
  // desaturate motors based on joyBias
  if (std::abs(throttle) + std::abs(turn) > 127) {
    int oldThrottle = throttle;
    int oldTurn = turn;
    throttle *= (1 - desaturateBias * std::abs(oldTurn / 127.0));
    turn *= (1 - (1 - desaturateBias) * std::abs(oldThrottle / 127.0));
    // ensure the sum of the two values is equal to 127
    // this check is necessary because of integer division
    if (std::abs(turn) + std::abs(throttle) == 126) {
      if (desaturateBias < 0.5) throttle += throttle < 0 ? -1 : 1;
      else turn += turn < 0 ? -1 : 1;
    }
  }

  float leftPower = throttle + turn;
  float rightPower = throttle - turn;

  return {leftPower, rightPower};
}

Velocities curvature(int throttle, int turn) {
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
  // Same direction?
  bool sameDirection = (prev * target > 0);

  double riseDelta = (type == MotionType::FORWARD) ? slew.risingThrottle : slew.risingAngle;
  double fallDelta = (type == MotionType::FORWARD) ? slew.fallingThrottle : slew.fallingAngle;

  // Increasing magnitude in same direction -> accelerating
  bool accelerating = sameDirection && (fabs(target) > fabs(prev));
  double maxDelta = accelerating ? riseDelta : fallDelta;
  if (delta > maxDelta) delta = maxDelta;
  if (delta < -maxDelta) delta = -maxDelta;
  prev = target;
  return prev + delta;
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
  if (fabs(input) < config.deadband) return 0;
  
  bool CONTROL_OVERRIDE = false;
  if (abs(controller.get_analog(ANALOG_RIGHT_X)) > config.joystickSpeedOverrideThreshold) {
    CONTROL_OVERRIDE = true;
  } else {
    CONTROL_OVERRIDE = false;
  }
  
  float speed = 
    normVel((leftDrivetrain.get_actual_velocity() + 
            rightDrivetrain.get_actual_velocity()) / 2);

  float retExpo = (speed > config.overrideSpeedMultiplier || CONTROL_OVERRIDE) 
                     ? config.overrideSpeedMultiplier : config.defaultSpeedMultiplier;

  double norm = input / 127.0;
  double linear = norm;
  double exponential = pow(fabs(norm), config.expoTurn) * ((norm >= 0) ? 1 : -1);
  double blended = 0.39 * linear + 0.42 * exponential;
  if (fabs(blended) >= 1) return 127 * ((norm >= 0) ? 1 : -1);
  return blended * retExpo;
}

void updateDrive(const DriveConfig& config) {
  const int throttle = controller.get_analog(ANALOG_LEFT_Y);
  const int turn = controller.get_analog(ANALOG_RIGHT_X);

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
    velocities = arcade(throttle, turn, config.desaturateBias);
  } else {
    velocities = curvature(throttle, turn);
  }

  // Multiply by 4.7244094488 to get rpm from voltage
  leftDrivetrain.move_velocity(velocities.leftVelocity * 4.7244094488);
  rightDrivetrain.move_velocity(velocities.rightVelocity * 4.7244094488);
}
} // namespace control
