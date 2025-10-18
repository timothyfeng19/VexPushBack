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
rotation odom = rotation(PORT21, true);;
vex::digital_out load = digital_out(Brain.ThreeWirePort.A);
vex::digital_out mid = digital_out(Brain.ThreeWirePort.B);
vex::digital_out park = digital_out(Brain.ThreeWirePort.C);

vex::controller Controller = controller(primary);

bool lpress = false;
bool apress = false;
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
    if (display_buffer == 500) {
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
        // under.setVelocity(-50, percent);
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
      park = false;
    }

    if (Controller.ButtonDown.pressing()) {
      park = true;
    }

    left_motors.spin(forward);
    right_motors.spin(forward);
    this_thread::sleep_for(20);
  }
}



void drive_pid(double pos, double target_deg) {
  double last_error = 0;
  double time_dif;
  double p = 5;
  double d = 4;
  double current_pos;
  double vel;
  double error;
  /*
  double t_error;
  double t_raw_error;
  double mult = 2;
  double current_head;
  */

  Drivetrain.setStopping(brake);
  left_motors.spin(forward);
  right_motors.spin(forward);
  odom.resetPosition();
  while (true) {
    /*
    current_head = imu.heading();
    t_raw_error = target_deg - current_head;
    if (t_raw_error > 180) {
      t_raw_error -= 360;
    }
    if (t_raw_error < -180) {
      t_raw_error += 360;
    }
    t_error = t_raw_error;
    */
    current_pos = odom.position(turns)*2.75*M_PI;
    error = pos - current_pos;
    time_dif = Brain.Timer.time(msec);
    vel = p * error - d * (last_error-error) / time_dif;

    left_motors.setVelocity(vel, percent);
    right_motors.setVelocity(vel, percent);

    /*
    if (t_error > 0) {
      right_motors.setVelocity(vel - mult * t_error, percent);
    } else if (t_error < 0) {
      left_motors.setVelocity(vel - mult * t_error, percent);
    }
    */

    last_error = error;
    Brain.Timer.clear();

    if (std::abs(error) < 0.5 && vel < 2) {
      break;
    }
    
    this_thread::sleep_for(20);
  }
  Drivetrain.stop();
}



void drive_pid1(double pos, double target_deg) {
  double last_error = 0;
  double time_dif;
  double p = 5;
  double d = 4;
  double current_pos;
  double vel;
  double error;
  /*
  double t_error;
  double t_raw_error;
  double mult = 2;
  double current_head;
  */

  Drivetrain.setStopping(brake);
  left_motors.spin(forward);
  right_motors.spin(forward);
  odom.resetPosition();
  while (true) {
    /*
    current_head = imu.heading();
    t_raw_error = target_deg - current_head;
    if (t_raw_error > 180) {
      t_raw_error -= 360;
    }
    if (t_raw_error < -180) {
      t_raw_error += 360;
    }
    t_error = t_raw_error;
    */
    current_pos = odom.position(turns)*2.75*M_PI;
    error = pos - current_pos;
    time_dif = Brain.Timer.time(msec);
    vel = p * error - d * (last_error-error) / time_dif;

    left_motors.setVelocity(vel, percent);
    right_motors.setVelocity(vel * 0.9, percent);

    /*
    if (t_error > 0) {
      right_motors.setVelocity(vel - mult * t_error, percent);
    } else if (t_error < 0) {
      left_motors.setVelocity(vel - mult * t_error, percent);
    }
    */

    last_error = error;
    Brain.Timer.clear();

    if (std::abs(error) < 0.5 && vel < 2) {
      break;
    }
    
    this_thread::sleep_for(20);
  }
  Drivetrain.stop();
}



void turn_pid(double target_deg) {
  double correct_buffer = 0;
  double last_error = 0;
  double time_dif;
  double p = 0.4;
  double d = 8;
  double current_head;
  double vel;
  double error;
  double raw_error;

  Drivetrain.setStopping(brake);
  Drivetrain.turn(right);
  while (true) {
    current_head = imu.heading();
    raw_error = target_deg - current_head;
    if (raw_error > 180) {
      raw_error -= 360;
    }
    if (raw_error < -180) {
      raw_error += 360;
    }
    error = raw_error;
    time_dif = Brain.Timer.time(msec);
    vel = p * error - d * (last_error - error) / time_dif;
    Drivetrain.setTurnVelocity(vel, percent);
    last_error = error;
    Brain.Timer.clear();

    if (std::abs(error) < 1 && std::abs(vel) < 2) {
      correct_buffer ++;
      if (correct_buffer > 2) {
        break;
      }
    } else {
      correct_buffer = 0;
    }
    this_thread::sleep_for(20);
  }
  Drivetrain.stop();
}



int auton_buffer = 0;
int auton_buffer2 = 0;

void auton() {
  imu.setHeading(0, degrees);
  all.setVelocity(100, percent);
  top.setVelocity(0, percent);
  all.spin(forward);
  drive_pid(37, 0);
  turn_pid(225);
  drive_pid(-9.5, 225);
  bottom.setVelocity(0, percent);
  top.setVelocity(50, percent);
  wait(0.3, seconds);
  top.setVelocity(0, percent);

  drive_pid1(48.5, 225);
  turn_pid(180);
  bottom.setVelocity(100, percent);
  load = true;
  wait(0.4, seconds);
  Drivetrain.setDriveVelocity(70, percent);
  Drivetrain.drive(forward);
  wait(1.3, seconds);
  Drivetrain.stop();
  turn_pid(182);
  Drivetrain.drive(reverse);
  wait(1.2, seconds);
  load = false;
  top.setVelocity(100, percent);
  /*
  waitUntil(colorsort.hue() < 20);
  Drivetrain.stop();
  drive_pid(10, 180);
  */
}

int main() {
  competition Competition = competition();
  Competition.autonomous(auton);
  Competition.drivercontrol(telop);
  return (0);
}
