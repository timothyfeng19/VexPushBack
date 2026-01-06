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
      all.setVelocity(-20, percent);
      //intake.setVelocity(-50, percent);
      all.spin(forward);
    } else {
      r = false;
      if (!l) {
        all.stop();
      }
    }

    if (controller_1.ButtonL2.pressing()) {
      l = true;
      all.setVelocity(25, percent);
      top.setVelocity(30, percent);
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

    
    load = true;
    driveTo(38, 1500, true);
    turnToAngle(-90, 500, true);
    wait(0.1, seconds);
    driveTo(15, 500, true);
    wait(1.5, seconds);
    driveTo(-2, 200, true);
    wait(0.3, seconds);
    driveTo(2, 300, true);
    wait(0.2, seconds);
    driveTo(-7, 500, true);
    load = false;
    turnToAngle(0, 800, true);
    wait(0.1, seconds);
    curveCircle(90, 11, 2000, true);
    driveTo(75, 3000, true);
    turnToAngle(180, 800, true);
    driveTo(8.5, 750, true);
    dsr(18.5, 800);
    wait(0.5, seconds);
    turnToAngle(90, 800, true);
    driveTo(-15, 1200, true);
    wing = true;
    all.spin(reverse);
    wait(0.3, seconds);
    all.spin(forward);
    load = true;
    wait(2.5, seconds);
    driveTo(20, 1000, true);
    wing = false;
    driveTo(16, 1000, true, 6);
    wait(1.5, seconds);
    driveTo(-2, 200, true);
    wait(0.2, seconds);
    driveTo(2, 200, true);
    wait(0.5, seconds);
    driveTo(-12.5, 1000, false);
    driveTo(-25, 1200, true, 7);
    wing = true;
    all.spin(reverse);
    wait(0.3, seconds);
    all.spin(forward);
    wait(2, seconds);

    //wait(0.2, seconds);

    driveTo(10, 1200, true);
    wing = false;
    turnToAngle(0, 800, true);
    driveTo(-95, 4000, true);
    dsr(19.75, 500);
    turnToAngle(90, 800, true);

    wait(0.1, seconds);
    driveTo(20, 1000, true, 6);
    wait(2, seconds);
    driveTo(-2, 200, true);
    wait(0.2, seconds);
    driveTo(2, 800, true);
    wait(0.5, seconds);
    driveTo(-7, 500, true);
    load = false;
    turnToAngle(180, 800, true);
    wait(0.1, seconds);
    curveCircle(-92, 11, 1500, true);
    driveTo(79, 3000, true);
    turnToAngle(0, 800, true);
    driveTo(8.5, 500, true);
    dsr(19.45, 250);
    wait(0.5, seconds);
    turnToAngle(-90.5, 1000, true);
    driveTo(-18, 1200, true);
    wing = true;
    all.spin(reverse);
    wait(0.2, seconds);
    all.spin(forward);
    load = true;
    wait(2.5, seconds);
    turnToAngle(-90, 500, true);
    //wing = false;
    driveTo(20, 1000, true);
    wing = false; //
    driveTo(16, 800, true, 6);
    wait(2, seconds);
    driveTo(-2, 200, true);
    wait(0.2, seconds);
    driveTo(2, 200, true);
    wait(0.5, seconds);
    //  driveTo(-10, 1000, false);
    //  turnToAngle(0, 800, true);
    //  dsr(18.6, 1000);
    //  turnToAngle(-90, 800, true);
    driveTo(-30, 1000, true, 7);
    wing = true;
    load = false;
    all.spin(reverse);
    wait(0.2, seconds);
    all.spin(forward);
    wait(2, seconds);
    
    
    left_chassis.setVelocity(100, percent);
    right_chassis.setVelocity(100, percent);
    left_chassis.spin(forward);
    right_chassis.spin(forward);
    wait(0.25, seconds);
    wing = false;
    left_chassis.setVelocity(100, percent);
    right_chassis.setVelocity(10, percent);
    wait(0.25, seconds);
    left_chassis.setVelocity(100, percent);
    right_chassis.setVelocity(100, percent);
    wait(1.7, seconds);
    left_chassis.setVelocity(0, percent);
    right_chassis.setVelocity(0, percent);
    left_chassis.stop();
    right_chassis.stop();
    /*
    driveTo(22, 1000, true);
    wing = false;
    turnToAngle(0, 800, true);
    load = false;
    dsr(18.6, 1500, 0.0005);
    driveTo(20, 800, true);
    turnToAngle(-90, 800, true);
    curveCircle(-10, 10, 2000);
    load = true;
    driveTo(35, 4000, true);
    load = false;
    */
}


int main() {
  competition Competition = competition();
  Competition.autonomous(auton);
  Competition.drivercontrol(telop);
  return (0);
}
