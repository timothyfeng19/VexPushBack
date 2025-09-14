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
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}

#pragma endregion VEXcode Generated Robot Configuration

#include "vex.h"
#include<cmath>

using namespace vex;

// Driving motors
vex::motor left_motor1 = motor(PORT1, ratio6_1, true);
vex::motor left_motor2 = motor(PORT2, ratio6_1, true);
vex::motor left_motor3 = motor(PORT3, ratio6_1, true);
vex::motor_group left_motors (left_motor1, left_motor2, left_motor3);

vex::motor right_motor1 = motor(PORT6, ratio6_1, false);
vex::motor right_motor2 = motor(PORT7, ratio6_1, false);
vex::motor right_motor3 = motor(PORT8, ratio6_1, false);
vex::motor_group right_motors (right_motor1, right_motor2, right_motor3);

vex::motor intake = motor(PORT11, false);
vex::motor under = motor(PORT12, true);
vex::motor over = motor(PORT13, true);
vex::motor top = motor(PORT14, true);
vex::motor_group suball (intake, top);
vex::motor_group all (intake, under, over, top);

// Colorsort sensor
vex::optical sensor = optical(PORT16);
vex::digital_out lock = digital_out(Brain.ThreeWirePort.A);
vex::digital_out lift = digital_out(Brain.ThreeWirePort.B);

vex::controller Controller = controller(primary);

bool lpress = false;
bool sort = true;
bool l = false;
bool r = false;

int main() {
  while (true) {
    left_motors.setVelocity(0, percent);
    right_motors.setVelocity(0, percent);
          
    int throttle = -Controller.Axis3.position();
    int steering = -Controller.Axis1.position();

    left_motors.setVelocity(throttle + steering, percent);
    right_motors.setVelocity(throttle - steering, percent);

    if (Controller.ButtonR1.pressing()) {
      r = true;
      all.setVelocity(65, percent);
      under.setVelocity(40, percent);
      if (!sort) {
        if (lock) {
          all.setVelocity(75, percent);
        }
        all.spin(forward);
      } else {
        suball.spin(forward);
        over.spin(reverse);
        under.spin(forward);
      }
    } else if (Controller.ButtonR2.pressing()) {
      r = true;
      all.setVelocity(75, percent);
      if (!sort) {
        all.spin(reverse);
      } else {
        suball.spin(reverse);
        over.spin(reverse);
        under.spin(forward);
      }
    } else {
      r = false;
      if (!l) {
      all.stop();
      }
    }

    if (Controller.ButtonL2.pressing()) {
      l = true;
      all.setVelocity(75, percent);
      intake.spin(forward);
      top.spin(reverse);
      over.setVelocity(40, percent);
      over.spin(forward);
      under.setVelocity(25, percent);
      under.spin(forward);
    } else {
      l = false;
      if (!r) {
      all.stop();
      }
    }

    if (Controller.ButtonL1.pressing()) {
      if (!lpress) {
        lock = !lock;
        lpress = true;
      }
    } else {
      lpress = false;
    }

    if (Controller.ButtonUp.pressing()) {
      lift = true;
    }

    if (Controller.ButtonDown.pressing()) {
      lift = false;
    }
    
    // test
    sort = Controller.ButtonB.pressing();

    left_motors.spin(forward);
    right_motors.spin(forward);

    this_thread::sleep_for(10);
  }
}
