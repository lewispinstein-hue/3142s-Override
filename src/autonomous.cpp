#include "setup.hpp"
void MotionViewAuton() {
chassis.setPose(0, 0, 0);
chassis.moveToPoint(-0.2, -0.7, 4000, {.maxSpeed = 127});
chassis.turnToHeading(0, 10000, {.maxSpeed = 127});
chassis.moveToPoint(-0.2, 22.9, 4000, {.maxSpeed = 127});
chassis.turnToHeading(273.181, 10000, {.maxSpeed = 127});
chassis.moveToPoint(-23.4, 23.4, 4000, {.maxSpeed = 127});
chassis.turnToHeading(135.556, 10000, {.maxSpeed = 127});
chassis.moveToPoint(-23.8, 0.1, 4000, {.maxSpeed = 127});
chassis.turnToHeading(47.784, 10000, {.maxSpeed = 127});
chassis.moveToPoint(-12.3, 11.1, 4000, {.maxSpeed = 127});
chassis.turnToHeading(177.338, 10000, {.maxSpeed = 127});
chassis.moveToPoint(-12.1, -5.2, 4000, {.maxSpeed = 127});
chassis.turnToHeading(60.222, 10000, {.maxSpeed = 127});
chassis.moveToPoint(0, -0.5, 4000, {.maxSpeed = 127});
chassis.turnToHeading(331.642, 10000, {.maxSpeed = 127});
}