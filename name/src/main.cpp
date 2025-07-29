
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
vex::motor left_motor2 = motor(PORT15, ratio6_1, true);
vex::motor_group left_motors (left_motor1, left_motor2);

vex::motor right_motor1 = motor(PORT6, ratio6_1, false);
vex::motor right_motor2 = motor(PORT7, ratio6_1, false);
vex::motor_group right_motors (right_motor1, right_motor2);

vex::motor intake = motor(PORT2, false);
vex::motor under = motor(PORT12, true);
vex::motor over = motor(PORT10, true);
vex::motor top = motor(PORT14, true);
vex::motor tippytop = motor(PORT3, true);
vex::motor_group suball (intake, top);
vex::motor_group all (intake, under, over, top, tippytop);

// Colorsort sensor
vex::optical sensor = optical(PORT20);
// Pneumatic block lock
vex::digital_out lock = digital_out(Brain.ThreeWirePort.A);

vex::controller Controller = controller(primary);

bool lpress = false;
bool sort = true;
bool l = false;
bool r = false;

bool color_sort_enabled = false;

bool red_detected = false;
bool blue_detected = false;
bool safety_lock = false;

int color_select = 1; //1 = blue, 0 = red

int direction_select = 0;


double expo_scale = 0.9; 

bool full_lock = false;

bool full_speed = false;


void update_stats(){
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 0);
  Controller.Screen.print(color_select == 1 ? "Blue" : "Red");
  Controller.Screen.setCursor(2, 0);
  Controller.Screen.print((color_sort_enabled ? "Auto Sort: On" : "Auto Sort: Off"));
  Controller.Screen.setCursor(3, 0);
  Controller.Screen.print((safety_lock ? "Safety Lock: On" : "Safety Lock: Off"));

  Brain.Screen.setFillColor(color_select == 1 ? 0 : 240/*240 : 0*/);
  Brain.Screen.drawRectangle(0, 0, 500, 500);
}
int main() {
  update_stats();
  while (true) {
    if(Controller.ButtonX.pressing()){
      full_lock = !full_lock;
      Controller.rumble(".");

      if(full_lock){
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 0);
        Controller.Screen.print("FULL LOCK");
        Controller.Screen.setCursor(2, 0);
        Controller.Screen.print("X to disable");
      }
      else{
        update_stats();
      }
    }

    if(full_lock){

      all.stop();
      left_motors.setVelocity(0, percent);
      right_motors.setVelocity(0, percent);
      all.setVelocity(0, percent);
    }

    if(!full_lock){

      red_detected = (sensor.color() == 16711680 || sensor.color() == 16744192);
      blue_detected = (sensor.color() == 255 || sensor.color() == -243679 || sensor.color() == 65280);

      direction_select = (blue_detected || red_detected) ? ((color_select == 1) ? (red_detected) : (blue_detected)) : direction_select;

      left_motors.setVelocity(0, percent);
      right_motors.setVelocity(0, percent);
      all.setVelocity(50, percent);
      





      if(Controller.ButtonLeft.pressing()){
          color_select = 1;
          update_stats();
      }
      if(Controller.ButtonRight.pressing()){
          color_select = 0;
          update_stats();
      }
      if(Controller.ButtonL1.pressing()){
        full_speed = true;
      }
      else{
        full_speed = false;
      }

      double left_input = Controller.Axis3.position();
      double right_input = Controller.Axis2.position();

      double left_norm = left_input / 100.0;
      double right_norm = right_input / 100.0;

      double left_scaled = expo_scale * pow(left_norm, 3) + (1 - expo_scale) * left_norm;
      double right_scaled = expo_scale * pow(right_norm, 3) + (1 - expo_scale) * right_norm;

      double safety_scale = safety_lock ? 0.1 : 1.0;

      left_motors.setVelocity(-left_scaled * (!full_speed ? 100 : 25) * safety_scale, percent);
      right_motors.setVelocity(-right_scaled * (!full_speed ? 100 : 25) * safety_scale, percent);
      if(Controller.ButtonUp.pressing()){
          color_sort_enabled = !color_sort_enabled;
          update_stats();
      }

      if(Controller.ButtonDown.pressing()){
          safety_lock = !safety_lock;
          update_stats();
      }


      
      if(Controller.ButtonR1.pressing()){
          all.spin(forward);
          top.spin(reverse);
          tippytop.spin(reverse);
          over.setVelocity(30, percent);


          if(Controller.ButtonB.pressing() || (direction_select == 1 && color_sort_enabled)){
              top.spin(forward);
          }
          if(Controller.ButtonA.pressing()){
              tippytop.spin(forward);
          }
      }
      else if(Controller.ButtonR2.pressing()){

          all.spin(reverse);
          top.spin(forward);
          tippytop.spin(forward);

          if(Controller.ButtonB.pressing() || (direction_select == 1 && color_sort_enabled)){
              top.spin(forward);
          }
          if(Controller.ButtonA.pressing()){
              tippytop.spin(forward);
          }
      }
      else{
          all.stop();
          top.stop();
          tippytop.stop();
      }

      left_motors.spin(forward);
      right_motors.spin(forward);
    }

    this_thread::sleep_for(5);
  }

}
