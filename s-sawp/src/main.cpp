#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "pid.h"
#include "motor-control.h"
#include "robot-config.h"
#include "utils.h"
#include "vex.h"
#include "dsr.h"

using namespace vex;

// Brain should be defined by default

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
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}

// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}


bool lpress = false;
bool apress = false;
bool upress = false;
bool hoard = false;

bool l = false;
bool r = false;
int display_buffer = 0;

void update() {
  controller_1.Screen.clearScreen();

  controller_1.Screen.setCursor(1,1);
  controller_1.Screen.print("Driver Control");

  controller_1.Screen.setCursor(2,1);
  if (hoard) {
    controller_1.Screen.print("Hoarding Enabled");
  } else {
    controller_1.Screen.print("Hoarding Disabled");
  }
  controller_1.Screen.setCursor(3,1);
  controller_1.Screen.print(Brain.Battery.capacity());
}

void telop() {
  update();
  while(true) {
    display_buffer += 1;
    if (display_buffer == 200) {
      update();
    }

    left_chassis.setStopping(coast);
    right_chassis.setStopping(coast);
    int left = controller_1.Axis3.position();
    int right = controller_1.Axis2.position();

    left_chassis.setVelocity(left, percent);
    right_chassis.setVelocity(right, percent);

    if (controller_1.ButtonR1.pressing()) {
      r = true;
      all.setVelocity(100, percent);
      if (hoard) {
        top.setVelocity(0, percent);
      }
      all.spin(forward);
    } else if (controller_1.ButtonR2.pressing()) {
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

    if (controller_1.ButtonL2.pressing()) {
      l = true;
      all.setVelocity(100, percent);
      all.spin(forward);
      top.spin(reverse);
    } else {
      l = false;
      if (!r) {
        all.stop();
      }
    }

    if (controller_1.ButtonL1.pressing()) {
      if(!lpress){
        wing = !wing;
        lpress = true;
        update();
      }
    } else {
      lpress = false;
    }

    if (controller_1.ButtonA.pressing()) {
      if (!apress) {
        load = !load;
        apress = true;
      }
    } else {
      apress = false;
    }

    if (controller_1.ButtonUp.pressing()) {
      if (!upress) {
        wing = !wing;
        upress = true;
      }
    } else {
      upress = false;
    }

    left_chassis.spin(forward);
    right_chassis.spin(forward);
    this_thread::sleep_for(20);
  }
}

// Removed PID functions

int auton_buffer = 0;
int auton_buffer2 = 0;

void auton() {
    inertial_sensor.setHeading(0, degrees);
    all.setVelocity(100, percent);
    all.spin(forward);
    driveTo(-38, 1400, true);
    load = true;
    dsr(18.52, 500);
    turnToAngle(-90, 600, true);
    driveTo(20, 800, true);
    wait(0.25, seconds);
    driveTo(-26.9, 550, true);
    driveTo(-1, 10, false, 4);
    load = false;
    wing = true;
    wait(1.2, seconds);
    turnToAngle(10, 800, true);
    wing = false;
    driveTo(7, 450, true);
    turnToAngle(0, 150, true);
    driveTo(3, 200, true, 9);
    driveTo(42.85, 1000, false);
    all.spin(reverse);
    load = true;
    driveTo(8.1, 300, true);
    all.spin(forward);
    wait(0.05, seconds);
    driveTo(-3.95, 200, true);
    load = false;
    turnToAngle(-45, 500, true);
    wait(0.05, seconds);
    driveTo(-26.9, 500, true);
    driveTo(-1, 10, false, 2);
    top.setVelocity(64, percent);
    top.spin(reverse);
    wait(1.4, seconds);
    turnToAngle(-45, 300, true);
    top.setVelocity(100, percent);
    driveTo(50.5, 2500, true);
    top.spin(forward);
    load = true;
    turnToAngle(-90, 500, true);
    driveTo(15, 750, true, 9);
    wait(0.25, seconds);
    driveTo(-26.5, 590, true);
    driveTo(-1, 10, false, 5);
    all.spin(forward);
    wing = true;
    all.spin(reverse);
    wait(0.2, seconds);
    all.spin(forward);
    wait(2, seconds);
}


int main() {
  competition Competition = competition();
  Competition.autonomous(auton);
  Competition.drivercontrol(telop);
  return (0);
}