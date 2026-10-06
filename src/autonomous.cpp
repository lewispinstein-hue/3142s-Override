#include "setup.hpp"
void MotionViewAuton() {
chassis.setPose(62, -0.5, 270);
chassis.moveToPoint(23, 0, 4000, {.forwards = true, .maxSpeed = 127});
chassis.turnToHeading(20, 1000);
chassis.moveToPoint(29.5, 25.5, 2000, {.forwards = true, .maxSpeed = 127});
chassis.turnToHeading(20, 1000);
chassis.moveToPoint(42, 55, 4000, {.forwards = true, .maxSpeed = 127});
chassis.turnToHeading(170, 1000);
chassis.moveToPoint(47, 26.5, 4000, {.forwards = true, .maxSpeed = 51});
chassis.turnToHeading(180, 1000, {}, false);
chassis.waitUntilDone();
chassis.moveToPoint(46.5, 66.5, 4000, {.forwards = false, .maxSpeed = 127});
chassis.moveToPoint(62, 49, 4000, {.forwards = false, .maxSpeed = 127});
chassis.waitUntilDone();
}