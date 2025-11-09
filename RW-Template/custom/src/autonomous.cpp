#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "motor-control.h"

// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }
bool lpress = false;
bool apress = false;
bool upress = false;
bool hoard = false;

bool l = false;
bool r = false;
int display_buffer = 0;

void exampleAuton() {
  // Use this for tuning linear and turn pid
  moveToPoint(10, 31, 1, 4000, true);
  Drivetrain.setTurnVelocity(25, percent);
  turnToAngle(-130, 1250, false);
  load = true;
  wait(1, seconds);
  all.spin(forward);
  all.setVelocity(100, percent);
  top.setVelocity(0, percent);
  Drivetrain.setDriveVelocity(35, percent);
  Drivetrain.driveFor(forward, 7, inches);
  wait(1, seconds);
  Drivetrain.driveFor(reverse, 0.5, inches);
  Drivetrain.driveFor(forward, 1, inches);
  wait(1, seconds);
  Drivetrain.driveFor(reverse, 15, inches);
  top.setVelocity(100, percent);
  wait(1, seconds);
  bottom.spin(reverse);
  wait(0.3, seconds);
  bottom.spin(forward);
  wait(2, seconds);
  top.setVelocity(0, percent);
  Drivetrain.driveFor(5, inches);
  turnToAngle(-90, 800, false);
  Drivetrain.driveFor(10, inches);
  turnToAngle(-95, 800, false);
  Drivetrain.driveFor(40, inches);
  turnToAngle(-90, 800, false);
  Drivetrain.driveFor(8, inches);
  turnToAngle(90, 800, false);
  Drivetrain.driveFor(10, inches);
  wait(1, seconds);
  Drivetrain.driveFor(reverse, 0.5, inches);
  Drivetrain.driveFor(forward, 1, inches);
  wait(1, seconds);
}

void exampleAuton2() {
  moveToPoint(24, 24, 1, 2000, false);
  moveToPoint(48, 48, 1, 2000, true);
  moveToPoint(24, 24, -1, 2000, true);
  moveToPoint(0, 0, 1, 2000, true);
  correct_angle = 0;
  driveTo(24, 2000, false, 8);
  turnToAngle(90, 800, false);
  turnToAngle(180, 800, true);
}