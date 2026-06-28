#include "drive.hpp"
#include "pros/motors.h"
#include "setup.hpp"

void initializeDr4bDebug() {
  mvlib::Logger& logger = mvlib::Logger::getInstance();
  logger.watch("DR4B Temp", LogLevel::INFO, WatchMode::onInterval, 5_mvS, 
  [&]() { return (dr4bMech.get_temperature(0) + dr4bMech.get_temperature(1)) / 2; },
  mvlib::LevelOverride<double>{
    .elevatedLevel = LogLevel::WARN,
    .predicate = PREDICATE(v > 45)
  });

  logger.watch("DR4B Watts", LogLevel::INFO, WatchMode::onInterval, 250_mvMs, 
  [&]() { return (dr4bMech.get_power(0) + dr4bMech.get_power(1)) / 2; });

  logger.watch("DR4B height", LogLevel::INFO, WatchMode::onInterval, 250_mvMs,
  [&]() { return (dr4bMech.get_position(0) + dr4bMech.get_position(1)) / 2; });
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  leftDrivetrain.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
  rightDrivetrain.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

  dr4bMech.tare_position_all(); // Dr4b MUST be lowered completely before starting
  auto& logger = mvlib::Logger::getInstance();
  mvlib::setOdom(&chassis);
  logger.setRobot({
    .leftDrivetrain = &leftDrivetrain,
    .rightDrivetrain = &rightDrivetrain
  });
  logger.setMinLogLevel(LogLevel::DEBUG);
  logger.setLogToSD(false);
  logger.setDefaultWatches({true, true, true});
  logger.setLoggingLocation("/beta/run#1.log");

  chassis.calibrate();
  chassis.setPose(0, 0, 0);
  logger.start();

  // logger.watch("Throttle", LogLevel::INFO, WatchMode::onInterval, 50_mvMs, 
  // [&]() { return controller.get_analog(ANALOG_LEFT_Y); } );
  // logger.watch("Turn", LogLevel::INFO, WatchMode::onInterval, 50_mvMs, 
  // [&]() { return controller.get_analog(ANALOG_RIGHT_X); } );

  // logger.watch("Battery %", LogLevel::INFO, WatchMode::onInterval, 2_mvS, 
  // [&]() { return pros::c::battery_get_capacity(); }, "%.1f");

  // logger.setPrintWatches(false);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

control::Slew slew{
  20,
  20,
  20,
  20
};

control::ExpoTurnConfig expoTurnConfig{
  4,
  120,
  100,
  127
};

control::DriveConfig config{
  .slew = slew,
  .expoTurnConfig = expoTurnConfig,
  .driveMode = control::DriveMode::ARCADE,
  .expoThrottle = 1.8,
  .deadband = 10,
  .desaturateBias = 0.5
};

void opcontrol() {
  auto& logger = mvlib::Logger::getInstance();
  logger.info("Opcontrol!");
  while (true) {
    dr4bHandle();
    control::updateDrive(config);
    if (controller.get_digital(DIGITAL_L1)) {
      logger.info("L1"); 
    }
    pros::delay(10);
  }
}
