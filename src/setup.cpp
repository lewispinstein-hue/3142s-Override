#include "setup.hpp"

// MotorGroups are {front, middle, back}
pros::MotorGroup leftDrivetrain(
  {1, 18},
  pros::v5::MotorGears::blue,
  pros::v5::MotorUnits::degrees
);

pros::MotorGroup rightDrivetrain(
  {-11, -10},
  pros::v5::MotorGears::blue,
  pros::v5::MotorUnits::degrees
);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// {right, left}
pros::MotorGroup dr4bMech({-4, -5});

// pros::adi::Pneumatics claw(11);
// pros::Motor clawPitch(12);

pros::Rotation horizontalOdom(-17);
pros::Rotation verticalOdom(12);
pros::IMU imu(15);


lemlib::TrackingWheel verticalTrackingWheel(
  &verticalOdom,
  lemlib::Omniwheel::NEW_2,
  0.915 // 0.915
);

lemlib::TrackingWheel horizontalTrackingWheel(
  &horizontalOdom,
  lemlib::Omniwheel::NEW_2,
  0.49 // 0.49
);

lemlib::OdomSensors sensors(
  &verticalTrackingWheel,
  nullptr,
  &horizontalTrackingWheel,
  nullptr,
  &imu
);

const double trackWidth = 10.75;
const double wheelbase = 10.5;

lemlib::Drivetrain drivetrain(
  &leftDrivetrain,
  &rightDrivetrain,
  trackWidth,
  lemlib::Omniwheel::NEW_275,
  24.0/36.0 * 600,
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
