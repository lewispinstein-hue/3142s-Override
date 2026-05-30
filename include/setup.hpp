#pragma once

#include "main.h"
#include "lemlib/api.hpp"

#define MVLIB_USE_SIMPLES
#include "mvlib/api.hpp"
#include "mvlib/Optional/lemlib.hpp"

extern pros::MotorGroup leftDrivetrain;
extern pros::MotorGroup rightDrivetrain;

extern pros::Rotation horizontalOdom;
extern pros::Rotation verticalOdom;
extern pros::IMU imu;

extern lemlib::Drivetrain drivetrain;
extern lemlib::Chassis chassis;

extern pros::Controller controller;

extern pros::MotorGroup liftMech;

extern pros::Motor claw;
extern pros::Motor clawPitch;
