#include "./../include/drive.hpp"
#include <algorithm>
#include <cmath>

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}


float curve(float input, float expo, int deadband = 10) {
  // return 0 if input is within deadzone
  if (fabs(input) <= deadband) return 0;
  // g is the output of g(x) as defined in the Desmos graph
  const float g = fabs(input) - deadband;
  // g127 is the output of g(127) as defined in the Desmos graph
  const float g127 = 127 - deadband;
  // i is the output of i(x) as defined in the Desmos graph
  const float i = pow(expo, g - 127) * g * sgn(input);
  // i127 is the output of i(127) as defined in the Desmos graph
  const float i127 = pow(expo, g127 - 127) * g127;

  // Min speed = 20
  return (127.0 - 20) / (127) * i * 127 / i127 + 20 * sgn(input);
}

namespace control {
Velocities arcade(float throttle, float turn, float expo, float desaturateBias) {
  // use drive curves if they have not been disabled
  throttle = std::round(curve(throttle, expo));
  turn = std::round(curve(turn, expo));

  // desaturate motors based on joyBias
  if (std::abs(throttle) + std::abs(turn) > 127) {
    float oldThrottle = throttle;
    float oldTurn = turn;
    throttle *= (1 - desaturateBias * std::abs(oldTurn / 127.0));
    turn *= (1 - (1 - desaturateBias) * std::abs(oldThrottle / 127.0));
    // ensure the sum of the two values is equal to 127
    // this check is necessary because of integer division
    if (std::abs(turn) + std::abs(throttle) == 126) {
      if (desaturateBias < 0.5) throttle += sgn(throttle);
      else turn += sgn(turn);
    }
  }

  float leftPower = throttle + turn;
  float rightPower = throttle - turn;

  float max = std::max(std::fabs(leftPower), std::fabs(rightPower)) / 127;
  if (max > 1) {
    leftPower /= max;
    rightPower /= max;
  }

  return {(int)leftPower, (int)rightPower};
}

Velocities curvature(float throttle, float turn, float expo, float desaturateBias) {
  // If we're not moving forwards change to arcade drive
  if (throttle == 0) {
    return arcade(throttle, turn, expo, desaturateBias);
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

  return {(int)leftPower, (int)rightPower};
}

float normVel(float rpm) {
  float v = (rpm / 4.7244094488); // Locked at blue gearset
  // If your using a different gearset, change this to (maxRPM / 127)
  return std::clamp(v, -127.0f, 127.0f);
}

float slewLimit(float target, float& prev, MotionType type, const Slew& slew) {

  if (target == prev) return target;
  float delta = target - prev;
  const bool sameDirection = (prev * target > 0);

  float riseDelta = (type == MotionType::FORWARD) ? slew.risingThrottle : slew.risingAngle;
  float fallDelta = (type == MotionType::FORWARD) ? slew.fallingThrottle : slew.fallingAngle;

  // Starting from zero is still acceleration; direction reversals use the falling limit.
  bool accelerating = (std::fabs(prev) == 0 || sameDirection) && (std::fabs(target) > std::fabs(prev));
  float maxDelta = accelerating ? riseDelta : fallDelta;
  if (delta > maxDelta) delta = maxDelta;
  if (delta < -maxDelta) delta = -maxDelta;
  prev += delta;
  return prev;
}

float expoThrottle(float input, float expoThrottle, float deadband) {
  if (fabs(input) < deadband) return 0;
  float norm = input / 127.0;
  float curved = pow(fabs(norm), expoThrottle);
  curved = curved * (norm >= 0 ? 1 : -1); 
  if (fabs(curved) >= 1) return 127 * ((norm >= 0) ? 1 : -1);
  return curved * 127.0;
}

float expoTurn(float input, const ExpoTurnConfig& config, const Drivetrain& dt) {
  const bool controlOverride =
    std::abs(dt.master->get_analog(ANALOG_RIGHT_X)) > config.joystickSpeedOverrideThreshold;

  const float turnMultiplier = 
    controlOverride ? config.overrideSpeedMultiplier
                    : config.defaultSpeedMultiplier;

  float norm = input / 127.0;
  float linear = norm;
  float exponential = pow(fabs(norm), config.expoTurn) * ((norm >= 0) ? 1 : -1);
  float blended = (0.39 * linear + 0.42 * exponential) / (0.39 + 0.42);
  if (fabs(blended) >= 1) return 127 * ((norm >= 0) ? 1 : -1);
  return blended * turnMultiplier;
}

void updateDrive(const DriveConfig& config) {
  int throttle = config.drivetrain.master->get_analog(ANALOG_LEFT_Y);
  int turn = config.drivetrain.master->get_analog(ANALOG_RIGHT_X);

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
    expoTurn(turn, config.expoTurnConfig, config.drivetrain),
    prevTurn,
    MotionType::TURN,
    config.slew
  );

  Velocities velocities;
  if (config.driveMode == DriveMode::ARCADE) {
    velocities = arcade(processedThrottle, processedTurn, config.expoTurnConfig.expoTurn, config.desaturateBias);
  } else {
    velocities = curvature(processedThrottle, processedTurn, config.expoTurnConfig.expoTurn, config.desaturateBias);
  }

  config.drivetrain.leftDrivetrain->move(velocities.leftVelocity);
  config.drivetrain.rightDrivetrain->move(velocities.rightVelocity);
}
} // namespace control
