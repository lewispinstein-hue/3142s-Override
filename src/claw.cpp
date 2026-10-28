#include "./../include/setup.hpp"
#include <cstdlib>

ClawState currentClawState = ClawState::CLOSED;
ClawState getClawState() {
  return currentClawState;  
}

void setClawState(ClawState state) {
  currentClawState = state;
}

constexpr int kClawOpenVelocity = 200;
constexpr int kClawKeepOpenPower = 2000;

constexpr int kClawCloseVelocity = 200;

void openClaw() {
  auto& logger = mvlib::Logger::getInstance();

  uint32_t startTime = pros::millis();
  claw.move_velocity(-kClawOpenVelocity);
  pros::delay(150);
  claw.move_voltage(-kClawKeepOpenPower);
  

  setClawState(ClawState::OPEN);
}

void closeClaw() {
  // Normal closing remains strong and continuous to retain objects.
  claw.move_velocity(kClawCloseVelocity);
  setClawState(ClawState::CLOSED);
}

void relaxClaw() {
  claw.move_velocity(0);
  setClawState(ClawState::RELAXED);
}

void setClawAction(ClawState action) {
  switch (action) {
    case ClawState::OPEN:
      openClaw();
      break;
    case ClawState::CLOSED:
      closeClaw();
      break;
    case ClawState::RELAXED:
      relaxClaw();
      break;
    default:
      break;
  }  
}

void controllerClawHandle() {
  ClawState currentState = getClawState();

  if (controller.get_digital_new_press(DIGITAL_L2)) {
    // Currently open, relax. Currently closed, open. Failed open, try again.
    setClawAction(currentState ==
          ClawState::OPEN ?
          ClawState::RELAXED :
            currentState == ClawState::OPEN_FAILED ?
            ClawState::OPEN :
            ClawState::OPEN);

  } else if (controller.get_digital_new_press(DIGITAL_L1)) {
    setClawAction(currentState == ClawState::CLOSED ? ClawState::RELAXED : ClawState::CLOSED);
  }
}
