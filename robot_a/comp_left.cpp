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
#include <cmath>

using namespace vex;

// Driving motors
vex::motor left_motor1 = motor(PORT1, ratio6_1, false);
vex::motor left_motor2 = motor(PORT2, ratio6_1, false);
vex::motor left_motor3 = motor(PORT3, ratio6_1, false);
vex::motor_group left_motors (left_motor1, left_motor2, left_motor3);

vex::motor right_motor1 = motor(PORT4, ratio6_1, true);
vex::motor right_motor2 = motor(PORT5, ratio6_1, true);
vex::motor right_motor3 = motor(PORT6, ratio6_1, true);
vex::motor_group right_motors (right_motor1, right_motor2, right_motor3);

drivetrain Drivetrain = drivetrain(left_motors, right_motors, 3.25, 12.75, 12, inches);

vex::motor intake = motor(PORT11, false);
vex::motor under = motor(PORT12, true); 
vex::motor over = motor(PORT13, false);
vex::motor top = motor(PORT14, true);
vex::motor_group all (intake, under, over, top);

vex::optical colorsort = optical(PORT15);
inertial imu(PORT16);
rotation odom = rotation(PORT21, true);
vex::digital_out hoard = digital_out(Brain.ThreeWirePort.A);
vex::digital_out park = digital_out(Brain.ThreeWirePort.B);
vex::digital_out load = digital_out(Brain.ThreeWirePort.C);

vex::controller Controller = controller(primary);

int team = 1; // 1 = red, 2 = blue

bool lpress = false;
bool apress = false;
bool bpress = false;
bool ypress = false;

bool sort_disabled = true;
bool sort = false;
bool l = false;
bool r = false;
int buffer = 0;
int sort_buffer;
bool unjam = false;
int display_buffer = 0;

void update() {
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1,1);
  if (team == 1) {
    Controller.Screen.print("(y) Red Driver Control");
  } else {
    Controller.Screen.print("(y) Blue Driver Control");
  }
  Controller.Screen.setCursor(2,1);
  Controller.Screen.print((sort_disabled ? "(b) Sort disabled" : "(b) Sort enabled"));
  Controller.Screen.setCursor(3,1);
  Controller.Screen.print(Brain.Battery.capacity());
}

void d() {
  update();
  while(true) {
    display_buffer += 1;
    if (display_buffer == 1000) {
      update();
    }

    left_motors.setVelocity(0, percent);
    right_motors.setVelocity(0, percent);
          
    int left = Controller.Axis3.position();
    int right = Controller.Axis2.position();

    left_motors.setVelocity(left, percent);
    right_motors.setVelocity(right, percent);

    if (colorsort.isNearObject() && !sort_disabled) {
      if ((colorsort.hue() < 150 || colorsort.hue() > 120) && team == 1) {
        sort = true;
      }
      if ((colorsort.hue() < 20 || colorsort.hue() > 350) && team == 2) {
        sort = true;
      }
    }

    if (sort) {
      sort_buffer += 1;
      if (sort_buffer >= 20) {
        sort_buffer = 0;
        sort = false;
      }
    }

    if (Controller.ButtonR1.pressing()) {
      r = true;
      all.setVelocity(100, percent);
      if (hoard) {
        under.setVelocity(-1, percent);
        if (unjam) {
          under.setVelocity(20, percent);
          buffer += 1;
        }
        if (buffer > 20) {
          buffer = 0;
          unjam = false;
        }
      }
      all.spin(forward);
      if (sort) {
        top.spin(reverse);
        over.spin(reverse);
      }
    } else if (Controller.ButtonR2.pressing()) {
      r = true;
      hoard = true;
      all.setVelocity(100, percent);
      all.spin(forward);
      intake.spin(reverse);
    } else {
      r = false;
      if (!l) {
      all.stop();
      }
    }

    if (Controller.ButtonL2.pressing()) {
      l = true;
      hoard = true;
      all.setVelocity(100, percent);
      all.spin(forward);
      top.spin(reverse);
    } else {
      l = false;
      if (!r) {
      all.stop();
      }
    }

    if (Controller.ButtonL1.pressing()) {
      if (!lpress) {
        hoard = !hoard;
        lpress = true;
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

    if (Controller.ButtonB.pressing()) {
      update();
      if (!bpress) {
        sort_disabled = !sort_disabled;
        bpress = true;
      }
    } else {
      bpress = false;
    }

    if (Controller.ButtonY.pressing()) {
      update();
      if (!ypress) {
        team ++;
        if (team == 3) {
          team = 1;
        }
        ypress = true;
      }
    } else {
      ypress = false;
    }

    if (Controller.ButtonUp.pressing()) {
      park = false;
    }

    if (Controller.ButtonDown.pressing()) {
      park = true;
    }

    if (Controller.ButtonX.pressing()) {
      unjam = true;
    }

    left_motors.spin(forward);
    right_motors.spin(forward);
    this_thread::sleep_for(20);
  }
}

void drive_pid(double pos) {
  double last_error = 0;
  double time_dif;
  double p = 2.3;
  double d = 0.1;
  double current_pos;
  double vel;
  double error;

  Drivetrain.setStopping(brake);
  Drivetrain.drive(forward);
  odom.resetPosition();
  while (true) {
    current_pos = odom.position(turns)*2*M_PI;
    error = pos - current_pos;
    time_dif = Brain.Timer.time(msec);
    vel = p * error - d * (last_error-error) / time_dif;
    Drivetrain.setDriveVelocity(vel, percent);
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

int timer_01 = 0;

void auton() {
  while(false){
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1,1);
    Controller.Screen.print(left_motor1.temperature(celsius));
    timer_01 += 1;
    Controller.Screen.setCursor(2,0);
    Controller.Screen.print(timer_01);
    wait(1000, msec);
  }

  //wait(2e1000, msec);


  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1,1);
  Controller.Screen.print("Auton Control");
  hoard = true;
  all.setVelocity(100, percent);
  under.setVelocity(-1, percent);
  all.spin(forward);
  left_motors.setVelocity(25, percent);
  right_motors.setVelocity(40, percent);
  left_motors.spin(forward);
  right_motors.spin(forward);
  wait(845, msec);
  Drivetrain.stop();
  turn_pid(45);
  under.setVelocity(100, percent);
  top.setVelocity(-60, percent);
  drive_pid(12.1);
  wait(1500, msec);

  hoard = true;
  top.setVelocity(100, percent);
  turn_pid(48);
  drive_pid(-49);
  turn_pid(180);
  load = true;
  wait(200, msec);

  Drivetrain.setDriveVelocity(80, percent);
  Drivetrain.drive(forward);
  wait(400, msec);
  Drivetrain.stop();
  Drivetrain.driveFor(reverse, 0.2, inches);
  Drivetrain.setDriveVelocity(20, percent);
  while (true) {
    auton_buffer ++;
    auton_buffer2 ++;
    all.setVelocity(100, percent);
    under.setVelocity(-1, percent);
    all.spin(forward);
    if (auton_buffer <= 11) {
      Drivetrain.drive(forward);
    }
    if (auton_buffer > 11) {
      if (auton_buffer < 20) {
        Drivetrain.drive(reverse);
      } else {
        auton_buffer = 0;
      }
    }
    if (auton_buffer2 >= 50) {
      break;
    }
    this_thread::sleep_for(20);
  }
  
  auton_buffer = 0;
  Drivetrain.stop();
  Drivetrain.setDriveVelocity(50, percent);
  Drivetrain.drive(reverse);
  wait(200, msec);
  Drivetrain.stop();

  load = false;
  turn_pid(0);

  //this_thread::sleep_for(1e19);

  wait(500, msec);
  hoard = false;
  Drivetrain.setDriveVelocity(25, percent);
  Drivetrain.drive(forward);
  all.setVelocity(100, percent);
  wait(500, msec);
  Drivetrain.setDriveVelocity(10, percent);
}

int main() {
  competition Competition = competition();
  Competition.autonomous(auton);
  Competition.drivercontrol(d);
  return (0);
}
