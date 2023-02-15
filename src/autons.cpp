#include "autons.hpp"
#include "main.h"
#include "pros/motors.hpp"
#include <random>

const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

bool alaunched = false;
bool alaunching = false;
bool astart = false;
bool adone = true;
int aminval = 1100;
int amaxval = 2100;
bool aendgame_state = false;

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor Intake(7);
pros::Motor launcher(4);
pros::ADIAnalogIn pm('A');
pros::ADIDigitalOut endgame('B');

///
// Constants
///

bool acheckLaunch() {
  if (adone && astart) {
    alaunching = true;
    adone = false;
    astart = false;
  }

  if (alaunching && pm.get_value() <= amaxval) {
    launcher.move_velocity(100);
  } else if (alaunching && pm.get_value() > aminval) {
    launcher.move_velocity(0);
    alaunching = false;
    alaunched = true;
  }

  if (alaunched && pm.get_value() >= aminval) {
    launcher.move_velocity(100);
  } else if (alaunched && pm.get_value() < aminval) {
    launcher.move_velocity(0);
    alaunched = false;
    adone = true;
  }

  return adone;
}

void waitUntilLaunched() {
  while (!acheckLaunch()) {
    pros::delay(20);
  }
}

// It's best practice to tune constants when the robot is empty and with heavier
// game objects, or with lifts up vs down. If the objects are light or the cog
// doesn't change much, then there isn't a concern here.

void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void one_mogo_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void two_mogo_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void exit_condition_defaults() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}

void modified_exit_condition() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}

void leftsideQWP() {
  chassis.set_tank(DRIVE_SPEED, DRIVE_SPEED);
  pros::delay(100);
  chassis.set_tank(0, 0);

  Intake.move_velocity(600);
  pros::delay(200);
  Intake.move_velocity(0);
  pros::delay(500);

  chassis.set_drive_pid(-1, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(0, DRIVE_SPEED);
  chassis.wait_drive();
  
  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-48, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(0, DRIVE_SPEED);
  chassis.wait_drive();
}

void rightsideQWP() {
  chassis.set_drive_pid(24, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(11, DRIVE_SPEED);
  chassis.wait_drive();

  Intake.move_relative(90, 600);
  pros::delay(500);

  chassis.set_drive_pid(-10, DRIVE_SPEED);
  chassis.wait_drive();

  astart = true;
  waitUntilLaunched();

  chassis.set_turn_pid(135, TURN_SPEED);
  chassis.wait_drive();

  Intake.move_velocity(600);

  chassis.set_drive_pid(60, DRIVE_SPEED / 2);
  chassis.wait_drive();

  Intake.move_velocity(0);

  chassis.set_turn_pid(-25, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(72, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(25, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(48, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(11, DRIVE_SPEED);
  chassis.wait_drive();

  Intake.move_relative(90, 600);
  pros::delay(500);

  chassis.set_drive_pid(-2, DRIVE_SPEED);
  chassis.wait_drive();
}

void blank() {
  chassis.set_tank(110, 100);
  pros::delay(100);
  chassis.set_tank(0, 0);

  Intake.move_velocity(600);
  pros::delay(1000);
  Intake.move_velocity(0);

  chassis.set_drive_pid(-6, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-5, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_tank(0, 0);

  pros::delay(1000);

  astart = true;
  waitUntilLaunched();
}
/*


///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // wait_until will wait until the robot gets to a desired position


  // When the robot gets to 6 inches, the robot will travel the remaining
distance at a max speed of 40 chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot
will go the remaining distance at 40 speed chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  // When the robot gets to -6 inches, the robot will travel the remaining
distance at a max speed of 40 chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_until(-6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot
will go the remaining distance at 40 speed chassis.wait_drive();
}



///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive


  chassis.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(12);

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();
}



///
// Auto that tests everything
///
void combining_movements() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, -45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Interference example
///
void tug (int attempts) {
  for (int i=0; i<attempts-1; i++) {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.set_drive_pid(-12, 127);
    chassis.wait_drive();

    // If failsafed...
    if (chassis.interfered) {
      chassis.reset_drive_sensor();
      chassis.set_drive_pid(-2, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees.
// If interfered, robot will drive forward and then attempt to drive backwards.
void interfered_example() {
 chassis.set_drive_pid(24, DRIVE_SPEED, true);
 chassis.wait_drive();

 if (chassis.interfered) {
   tug(3);
   return;
 }

 chassis.set_turn_pid(90, TURN_SPEED);
 chassis.wait_drive();
}



// . . .
// Make your own autonomous functions here!
// . . .
*/