#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller controller_1 = controller(primary);

// IMPORTANT: Remember to modify the example motors according to the guide. 
// Also remember to add respective device declarations to custom/include/robot-config.h
// Format: motor(port, gearSetting, reversed)
// gearSetting is one of the following: ratio36_1(red), ratio18_1(green), ratio6_1(blue)
// all chassis motors should be reversed appropriately so that they spin vertical when given a positive voltage input
// such as driveChassis(12, 12)
motor left_chassis1 = motor(PORT1, ratio6_1, true);
motor left_chassis2 = motor(PORT2, ratio6_1, true);
motor left_chassis3 = motor(PORT3, ratio6_1, true);
motor_group left_chassis = motor_group(left_chassis1, left_chassis2, left_chassis3);
motor right_chassis1 = motor(PORT4, ratio6_1, false);
motor right_chassis2 = motor(PORT5, ratio6_1, false);
motor right_chassis3 = motor(PORT6, ratio6_1, false);
motor_group right_chassis = motor_group(right_chassis1, right_chassis2, right_chassis3);
drivetrain Drivetrain = drivetrain(left_chassis, right_chassis, 3.17, 6.57, 1.5, inches);

inertial inertial_sensor = inertial(PORT10);
//optical colorsort = optical(PORT11);
//distance example_distance_sensor = distance(PORT14);



// Format is rotation(port, reversed)
// just set these to random ports if you don't use tracking wheels
rotation horizontal_tracker = rotation(PORT13, true);
rotation vertical_tracker = rotation(PORT19, true);

// game specific devices for push back
digital_out load = digital_out(Brain.ThreeWirePort.B);
digital_out wing = digital_out(Brain.ThreeWirePort.A);
motor intake = motor(PORT7, false);
motor middle = motor(PORT8, false);
motor top = motor(PORT9, false);
motor_group all (intake, middle, top);
distance left_dist = (PORT11);
distance right_dist = (PORT12);

// ============================================================================
// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
// ============================================================================

// Distance between the middles of the left and right wheels of the drive (in inches)
double distance_between_wheels = 10.28125;

// motor to wheel gear ratio * wheel diameter (in inches) * pi
double wheel_distance_in = (36.0 / 48.0) * 3.17 * M_PI;

// PID Constants for movement
// distance_* : Linear PID for straight driving
// turn_*     : PID for turning in place
// heading_correction_* : PID for heading correction during linear movement
double distance_kp = 0.6, distance_ki = 0, distance_kd = 1;
double turn_kp = 0.26, turn_ki = 0, turn_kd = 1.95;
double heading_correction_kp = 0.4, heading_correction_ki = 0, heading_correction_kd = 0;

// Enable or disable the use of tracking wheels
bool using_horizontal_tracker = false;  // Set to true if a horizontal tracking wheel is installed and used for odometry
bool using_vertical_tracker = false;   // Set to true if a vertical tracking wheel is installed and used for odometry

// IGNORE THESE IF YOU ARE NOT USING TRACKING WHEELS
// These comments are in the perspective of a top down view of the robot when the robot is facing vertical
// Vertical distance from the center of the bot to the horizontal tracking wheel (in inches, positive is when the wheel is behind the center)
double horizontal_tracker_dist_from_center = 2.71875;
// Horizontal distance from the center of the bot to the vertical tracking wheel (in inches, positive is when the wheel is to the right of the center)
double vertical_tracker_dist_from_center = -0.03125;
double horizontal_tracker_diameter = 1.975; // Diameter of the horizontal tracker wheel (in inches)
double vertical_tracker_diameter = 1.975; // Diameter of the vertical tracker wheel (in inches)

// ============================================================================
// ADVANCED TUNING (OPTIONAL)
// ============================================================================

bool heading_correction = true; // Use heading correction when the bot is stationary

// Set to true for more accuracy and smoothness, false for more speed
bool dir_change_start = true;   // Less accel/decel due to expecting direction change at start of movement
bool dir_change_end = true;     // Less accel/decel due to expecting direction change at end of movement

double min_output = 10; // Minimum output voltage to motors while chaining movements

// Maximum allowed change in voltage output per 10 msec during movement
double max_slew_accel_fwd = 24;
double max_slew_decel_fwd = 24;
double max_slew_accel_rev = 24;
double max_slew_decel_rev = 24;

// Prevents too much slipping during boomerang movements
// Decrease if there is too much drifting and inconsistency during boomerang
// Increase for more speed during boomerang
double chase_power = 2;

// ============================================================================
// DO NOT CHANGE ANYTHING BELOW
// ============================================================================

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // nothing to initialize
}