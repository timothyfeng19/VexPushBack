#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "pid.h"
#include "motor-control.h"
#include "utils.h"
#include "vex.h"


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

    if (controller_1.ButtonL1.pressing()) {
      if (!lpress) {
        hoard = !hoard;
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
  top.setVelocity(0, percent);
  all.spin(forward);
  Drivetrain.setDriveVelocity(40, percent);
  top.setVelocity(0, percent);
  driveTo(10, 1500, true);
  turnToAngle(-35, 800, true);
  driveTo(15, 2000, true);
  driveTo(-3, 400, true);
  turnToAngle(-135, 800, true);
  driveTo(-16, 1000, true);
  mid = true;
  top.setVelocity(100, percent);
  wait(0.6, seconds);
  mid = false;
  top.setVelocity(0, percent);
  driveTo(47, 4500, true);
  turnToAngle(-180, 1000, true);
  load = true;
  wait(0.5, seconds);
  driveTo(15, 1500, true);
  wait(0.1, seconds);
  driveTo(-0.5, 100, true);
  driveTo(1, 200, true);
  wait(0.1, seconds);
  driveTo(-40, 2000, true);
  top.setVelocity(100, percent);
}

int main() {
  competition Competition = competition();
  Competition.autonomous(auton);
  Competition.drivercontrol(telop);
  return (0);
}