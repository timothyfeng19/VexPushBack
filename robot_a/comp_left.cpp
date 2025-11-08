#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;

// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS

// Robot configuration code.

// generating and setting random seed
void initializeRandomSeed() {
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}

void vexcodeInit() {
  // Initializing random seed.
  initializeRandomSeed(); 
}

// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}

#pragma endregion VEXcode Generated Robot Configuration

#include <cmath>

using namespace vex;

// Driving motors 
vex::motor left_motor1 = motor(PORT1, ratio6_1, true);
vex::motor left_motor2 = motor(PORT2, ratio6_1, true);
vex::motor left_motor3 = motor(PORT3, ratio6_1, true);
vex::motor_group left_motors (left_motor1, left_motor2, left_motor3);

vex::motor right_motor1 = motor(PORT4, ratio6_1, false);
vex::motor right_motor2 = motor(PORT5, ratio6_1, false);
vex::motor right_motor3 = motor(PORT6, ratio6_1, false);
vex::motor_group right_motors (right_motor1, right_motor2, right_motor3);

drivetrain Drivetrain = drivetrain(left_motors, right_motors, 3.25, 12.75, 12, inches);

vex::motor bottom = motor(PORT7, false);
vex::motor top = motor(PORT8, true);
vex::motor_group all (bottom, top);

vex::optical colorsort = optical(PORT10);
inertial imu(PORT9);
rotation odom = rotation(PORT21, true);
vex::digital_out load = digital_out(Brain.ThreeWirePort.A);
vex::digital_out mid = digital_out(Brain.ThreeWirePort.B);
vex::digital_out wing = digital_out(Brain.ThreeWirePort.C);

vex::controller Controller = controller(primary);

bool lpress = false;
bool apress = false;
bool upress = false;
bool hoard = false;

bool l = false;
bool r = false;
int display_buffer = 0;

void update() {
  Controller.Screen.clearScreen();

  Controller.Screen.setCursor(1,1);
  Controller.Screen.print("Driver Control");

  Controller.Screen.setCursor(2,1);
  if (hoard) {
    Controller.Screen.print("Hoarding Enabled");
  } else {
    Controller.Screen.print("Hoarding Disabled");
  }
  Controller.Screen.setCursor(3,1);
  Controller.Screen.print(Brain.Battery.capacity());
}

void telop() {
  update();
  while(true) {
    display_buffer += 1;
    if (display_buffer == 200) {
      update();
    }

    left_motors.setVelocity(0, percent);
    right_motors.setVelocity(0, percent);
          
    int left = Controller.Axis3.position();
    int right = Controller.Axis2.position();

    left_motors.setVelocity(left, percent);
    right_motors.setVelocity(right, percent);

    if (Controller.ButtonR1.pressing()) {
      r = true;
      all.setVelocity(100, percent);
      if (hoard) {
        top.setVelocity(0, percent);
      }
      all.spin(forward);
    } else if (Controller.ButtonR2.pressing()) {
      r = true;
      if (hoard) {
        hoard = false;
        update();
      }
      hoard = false;
      all.setVelocity(-100, percent);
      all.spin(forward);
    } else {
      r = false;
      if (!l) {
        all.stop();
      }
    }

    if (Controller.ButtonL2.pressing()) {
      l = true;
      if (hoard) {
        hoard = false;
        update();
      }
      hoard = false;
      mid = true;
      all.setVelocity(100, percent);
      all.spin(forward);
    } else {
      mid = false;
      l = false;
      if (!r) {
        all.stop();
      }
    }

    if (Controller.ButtonL1.pressing()) {
      if (!lpress) {
        hoard = !hoard;
        lpress = true;
        update();
      }
    } else {
      lpress = false;
    }

    if (Controller.ButtonA.pressing()) {
      if (!apress) {
        load = !load;
        apress = true;
      }
    } else {
      apress = false;
    }

    if (Controller.ButtonUp.pressing()) {
      if (!upress) {
        wing = !wing;
        upress = true;
      }
    } else {
      upress = false;
    }

    left_motors.spin(forward);
    right_motors.spin(forward);
    this_thread::sleep_for(20);
  }
}

// Removed PID functions

int auton_buffer = 0;
int auton_buffer2 = 0;

void auton() {
  imu.setHeading(0, degrees);
  all.setVelocity(100, percent);
  top.setVelocity(0, percent);
  all.spin(forward);
  Drivetrain.setDriveVelocity(30, percent);
  top.setVelocity(0, percent);
  Drivetrain.driveFor(forward, 3, inches);
  Drivetrain.setTurnVelocity(30, percent);
  Drivetrain.turnFor(left, 11, degrees);
  Drivetrain.driveFor(forward, 12, inches);
  Drivetrain.turnFor(left, 30, degrees);
  Drivetrain.driveFor(reverse, 7, inches);
  top.setVelocity(100, percent);
  wait(0.2, seconds);
  top.setVelocity(0, percent);
  Drivetrain.setDriveVelocity(40, percent);
  Drivetrain.driveFor(forward, 27, inches);
  Drivetrain.turnFor(left, 15, degrees);
  load = true;
  wait(0.5, seconds);
  Drivetrain.driveFor(forward, 7, inches);
  wait(2, seconds);
  //Drivetrain.turnFor(left, 3, degrees);
  Drivetrain.driveFor(reverse, 15, inches);
  all.setVelocity(100, percent);
  top.setVelocity(100, percent);
  wait(1.5, seconds);
  Drivetrain.driveFor(forward, 5, inches);
}

int main() {
  competition Competition = competition();
  Competition.autonomous(auton);
  Competition.drivercontrol(telop);
  return (0);
}
