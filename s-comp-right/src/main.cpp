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

    driveTo(12, 2000, true);
    turnToAngle(45, 800, true);
    driveTo(14.5, 2000, false);
    load = true;
    driveTo(4, 1000, true, 3);
    load = false;
    wait(0.1, seconds);
    all.spin(reverse);
    driveTo(-5.75, 100, true);
    all.spin(forward);
    turnToAngle(120, 900, true);
    driveTo(24.1, 1400, true);
    load = true;
    //turnToAngle(-90, 800, true);
    //dsr(18.5, 300);
    turnToAngle(180, 900, true);
    driveTo(23, 1200, true);
    wait(0.22, seconds);
    driveTo(-18, 800, false, 10);
    driveTo(-5, 200, false, 4);
    driveTo(-1, 10, false, 5);
    wing = true;
    all.spin(reverse);
    wait(0.2, seconds);
    all.spin(forward);
    wait(2.5, seconds);
    wing = true;
    driveTo(7, 500, true);
    turnToAngle(-90, 800, true);
    driveTo(13.15, 800, true);
    dsr(31.65, 500);
    wing = false;
    turnToAngle(-167, 800, true);
    driveTo(-15, 1000, false, 5);
    turnToAngle(180, 200, false, 5);
    driveTo(-6, 1000, true, 5);
    driveTo(-1, 2000, false, 1);
}


int main() {
  competition Competition = competition();
  Competition.autonomous(auton);
  Competition.drivercontrol(telop);
  return (0);
}