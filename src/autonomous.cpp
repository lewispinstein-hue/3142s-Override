#include "setup.hpp"
void MotionViewAuton() {
auto& logger = mvlib::Logger::getInstance();
const uint32_t startTime = pros::millis();
chassis.setPose(0, 0, 0);
chassis.moveToPose(37, 40.5, 110, 4000, {.forwards = true, .lead = 0.638, .maxSpeed = 127, .minSpeed = 0}, false);
chassis.turnToHeading(110, 2000, {.maxSpeed = 127, .minSpeed = 0}, false);
chassis.moveToPose(11.5, 34, 35, 4000, {.forwards = false, .lead = 0.465, .maxSpeed = 127, .minSpeed = 0}, false);
chassis.moveToPose(9.5, 5, 330, 4000, {.forwards = false, .lead = 0.519, .maxSpeed = 127, .minSpeed = 0}, false);
chassis.turnToHeading(330, 2000, {.maxSpeed = 127, .minSpeed = 0}, false);
chassis.moveToPose(23, -1, 50, 4000, {.forwards = false, .lead = 0.728, .maxSpeed = 127, .minSpeed = 0}, false);
chassis.turnToHeading(50, 2000, {.maxSpeed = 127, .minSpeed = 0}, false);
chassis.moveToPose(0, 0, 5, 4000, {.forwards = false, .lead = 0.149, .maxSpeed = 100, .minSpeed = 0}, false);
chassis.turnToHeading(5, 2000, {.maxSpeed = 100, .minSpeed = 0}, false);
logger.info("Auton path completed in %dms", startTime - pros::millis());
}