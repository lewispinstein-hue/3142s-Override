#include "setup.hpp"
#include <cstdlib>

namespace {
uint32_t dr4bLastStallTime = 0;
double dr4bLastPosition = 0;
enum class StallDir {
  NONE,
  UP,
  DOWN
};

StallDir stallDir = StallDir::NONE;

bool getMotorStalled(pros::MotorGroup* motor, uint32_t mvThreshold = 2500,
                     float velocityThreshold = 5, uint8_t index = 0) {
  return (std::fabs(motor->get_actual_velocity(index)) < velocityThreshold &&
          motor->get_current_draw(index) > mvThreshold);
}
} // namespace

/**
 * @brief dr4bHandle
 * Instead of using buttons for up/down, we want to use a joystick.
 * If R1 is held, we transform the analog right Y into the movement of the dr4b
 * We use a combination of the velocity and current to detect if the dr4b has reached the end of it's motion
 * 
 */
void dr4bHandle() {
  static auto& logger = mvlib::Logger::getInstance();
  if (!controller.get_digital(DIGITAL_R1)) {
    dr4bMech.move_absolute(dr4bLastPosition, 90);
    dr4bLastStallTime = 0;
    return;
  }

  int rightY = controller.get_analog(ANALOG_RIGHT_Y);
  if (std::abs(rightY) < 10) rightY = 0;

  if ((stallDir == StallDir::UP && rightY <= 0) ||
      (stallDir == StallDir::DOWN && rightY >= 0)) {
    stallDir = StallDir::NONE;
  }

  if ((stallDir == StallDir::UP && rightY > 0) ||
      (stallDir == StallDir::DOWN && rightY < 0)) {
    dr4bMech.move(0);
    return;
  }

  bool dr4bStalled = false;
  for (uint8_t i = 0; i < dr4bMech.size(); i++) {
    if (getMotorStalled(&dr4bMech, 2500, 5, i)) {
      dr4bStalled = true;
      break;
    }
  }

  if (dr4bStalled) {
    logger.info("Dr4b Stalled");
    if (dr4bLastStallTime == 0) dr4bLastStallTime = pros::millis();

    if (pros::millis() - dr4bLastStallTime > 50) {
      dr4bMech.move(0);
      stallDir = (rightY > 0) ? StallDir::UP : StallDir::DOWN;
      return;
    }
  } else {
    dr4bLastStallTime = 0;
  }

  dr4bMech.move(rightY);

  dr4bLastPosition = (dr4bMech.get_position(0) + dr4bMech.get_position(1)) / 2;
}
