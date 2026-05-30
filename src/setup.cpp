#include "setup.hpp"

// MotorGroups are {front, middle, back}
pros::MotorGroup leftDrivetrain(
  {-1, -2, -3},
  pros::v5::MotorGears::blue,
  pros::v5::MotorUnits::degrees
);

pros::MotorGroup rightDrivetrain(
  {4, 5, 6},
  pros::v5::MotorGears::blue,
  pros::v5::MotorUnits::degrees
);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup liftMech({9, 10});

pros::Motor claw(11);
pros::Motor clawPitch(12);

pros::Rotation horizontalOdom(7);
pros::Rotation verticalOdom(8);
pros::IMU imu(10);


lemlib::TrackingWheel verticalTrackingWheel(
  &verticalOdom,
  lemlib::Omniwheel::NEW_275,
  0.5
);

lemlib::TrackingWheel horizontalTrackingWheel(
  &horizontalOdom,
  lemlib::Omniwheel::NEW_275,
  -0.5
);

lemlib::OdomSensors sensors(
  &verticalTrackingWheel,
  nullptr,
  &horizontalTrackingWheel,
  nullptr,
  &imu
);

const double drivetrainThickness = 3;
const double drivetrainSeparationDistance = 14.5;
const double wheelToInsideOfDrivetrainDistance = 1.75;
const double trackWidth = (drivetrainSeparationDistance - (drivetrainThickness * 2) + (wheelToInsideOfDrivetrainDistance * 2));

lemlib::Drivetrain drivetrain(
  &leftDrivetrain,
  &rightDrivetrain,
  trackWidth,
  lemlib::Omniwheel::OLD_325,
  360,
  2
);

lemlib::ControllerSettings lateralPID(
  10,  // proportional gain (kP)
  0,   // integral gain (kI)
  3,   // derivative gain (kD)
  3,         // anti windup
  1,          // small error range, in inches
  100, // small error range timeout, in milliseconds
  3,          // large error range, in inches
  500, // large error range timeout, in milliseconds
  0                 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularPID(
  2,   // proportional gain (kP)
  0,   // integral gain (kI)
  10,  // derivative gain (kD)
  3,         // anti windup
  1,          // small error range, in inches
  100, // small error range timeout, in milliseconds
  3,          // large error range, in inches
  500, // large error range timeout, in milliseconds
  0                // maximum acceleration (slew)
);

lemlib::Chassis chassis(
  drivetrain,
  lateralPID,
  angularPID,
  sensors
);
