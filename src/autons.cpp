#include "autons.hpp"
#include "main.h"
#include "pros/adi.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <future>
#include <random>

const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

bool aendgame_state = false;
int aminval = 1040;
int amaxval = 2100;
bool fin = false;

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor Intake(7);
pros::Motor launcher(4);
pros::ADIDigitalOut endgame('G');

///
// Constants
///

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

void launche() {
  launcher.move_velocity(100);
  pros::delay(2000);
  launcher.move_velocity(0);
}

void leftsideQWP() {
  chassis.set_tank(DRIVE_SPEED, DRIVE_SPEED);
  pros::delay(100);
  chassis.set_tank(0, 0);

  Intake.move_velocity(600);
  pros::delay(275);
  Intake.move_velocity(0);
  pros::delay(500);

  chassis.set_drive_pid(-6, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-56, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-40, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-2, DRIVE_SPEED);
  chassis.wait_drive();

  launche();
  
  chassis.set_drive_pid(5, DRIVE_SPEED);
  chassis.wait_drive();
  
  chassis.set_turn_pid(-135, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(58, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(7, DRIVE_SPEED);
  chassis.wait_drive();

  Intake.move_velocity(600);
  pros::delay(300);
  Intake.move_velocity(0);
  pros::delay(500);
}

void rightsideQWP() {
  chassis.set_drive_pid(20, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(3, DRIVE_SPEED);
  chassis.wait_drive();

  Intake.move_velocity(600);
  pros::delay(200);
  Intake.move_velocity(0);
  pros::delay(500);

  chassis.set_drive_pid(-6, DRIVE_SPEED);
  chassis.wait_drive();
  
  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-56, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(135, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-8, DRIVE_SPEED);
  chassis.wait_drive();

  launche();
  
  chassis.set_drive_pid(8, DRIVE_SPEED);
  chassis.wait_drive();
  
  chassis.set_turn_pid(225, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(58, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(180, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(4.5, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(270, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(180, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(2, DRIVE_SPEED);
  chassis.wait_drive();

  Intake.move_velocity(600);
  pros::delay(400);
  Intake.move_velocity(0);
  pros::delay(500);
}

void prog() {
  // FIRST ROLLER
  chassis.set_tank(DRIVE_SPEED, DRIVE_SPEED);
  pros::delay(100);
  chassis.set_tank(0, 0);

  Intake.move_velocity(600);
  pros::delay(275);
  Intake.move_velocity(0);
  pros::delay(500);

  chassis.set_drive_pid(-2, DRIVE_SPEED);
  chassis.wait_drive();

  // PATH TO SHOOT
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-66, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(85, TURN_SPEED);
  chassis.wait_drive();

  launche();

  chassis.set_drive_pid(-45.6, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-78, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  // SECOND ROLLER
  chassis.set_drive_pid(2, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_tank(DRIVE_SPEED, DRIVE_SPEED);
  pros::delay(100);
  chassis.set_tank(0, 0);

  Intake.move_velocity(600);
  pros::delay(275);
  Intake.move_velocity(0);
  pros::delay(500);

  chassis.set_drive_pid(-2, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-33.6, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-180, TURN_SPEED);
  chassis.wait_drive();
  
  // THIRD ROLLER
  chassis.set_drive_pid(2, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_tank(DRIVE_SPEED, DRIVE_SPEED);
  pros::delay(100);
  chassis.set_tank(0, 0);

  Intake.move_velocity(600);
  pros::delay(275);
  Intake.move_velocity(0);
  pros::delay(500);

  chassis.set_drive_pid(-2, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-102, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(102, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  // FOURTH ROLLER
  chassis.set_drive_pid(2, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_tank(DRIVE_SPEED, DRIVE_SPEED);
  pros::delay(100);
  chassis.set_tank(0, 0);

  Intake.move_velocity(600);
  pros::delay(275);
  Intake.move_velocity(0);
  pros::delay(500);

  chassis.set_drive_pid(-2, DRIVE_SPEED);
  chassis.wait_drive();

  // ENDGAME LINEUP
  chassis.set_turn_pid(135, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-16.8, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(225, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(12, DRIVE_SPEED);
  chassis.wait_drive();

  endgame.set_value(true);
  pros::delay(1000);

  chassis.set_drive_pid(-12, DRIVE_SPEED);
  chassis.wait_drive();
}

void blank() {}
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