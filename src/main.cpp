#include "main.h"
#include "autons.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <charconv>
#include <future>
#include <ios>
#include <iostream>
#include <string>

bool launched = true;
bool launching = false;
bool done = false;
bool endgame_state = false;
int minval = 1040;
int maxval = 2100;

// Chassis constructor
Drive chassis(
    // Left Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    {-13, 18, -12}

    // Right Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    ,
    {1, -3, 2}

    // IMU Port
    ,
    14

    // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
    //    (or tracking wheel diameter)
    ,
    3.25

    // Cartridge RPM
    //   (or tick per rotation if using tracking wheels)
    ,
    600

    // External Gear Ratio (MUST BE DECIMAL)
    //    (or gear ratio of tracking wheel)
    // eg. if your drive is 84:36 where the 36t is powered, your RATIO would
    // be 2.333. eg. if your drive is 36:60 where the 60t is powered, your RATIO
    // would be 0.6.
    ,
    ((float)60) / 30
    // Uncomment if using tracking wheels
    /*
    // Left Tracking Wheel Ports (negative port will reverse it!)
    // ,{1, 2} // 3 wire encoder
    // ,8 // Rotation sensor

    // Right Tracking Wheel Ports (negative port will reverse it!)
    // ,{-3, -4} // 3 wire encoder
    // ,-9 // Rotation sensor
    */

    // Uncomment if tracking wheels are plugged into a 3 wire expander
    // 3 Wire Port Expander Smart Port
    // ,1
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  // ez::print_ez_template();
  chassis.set_drive_brake(pros::E_MOTOR_BRAKE_COAST);

  pros::delay(
      500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(
      true); // Enables modifying the controller curve with buttons on the
             // joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(
      0, 0); // Defaults for curve. If using tank, only the first parameter is
             // used. (Comment this line out if you have an SD card!)
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants
                             // from autons.cpp!

  // These are already defaulted to these buttons, but you can change the
  // left/right curve buttons here! chassis.set_left_curve_buttons
  // (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If
  // using tank, only the left side is used.
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,
  // pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
      Auton("LeftSide qualifier", leftsideQWP),
      Auton("RightSide qualifier", rightsideQWP),
      Auton("Blank", blank),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}
// jonjonbad@code

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.reset_pid_targets();               // Resets PID targets to 0
  chassis.reset_gyro();                      // Reset gyro position to 0
  chassis.reset_drive_sensor();              // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD);
  ez::as::auton_selector
      .call_selected_auton(); // Calls selected auton from autonomous selector.
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  pros::Controller controller(pros::E_CONTROLLER_MASTER);
  pros::Motor Intake(7);
  pros::Motor launcher(4);
  pros::ADIAnalogIn pm('F');
  pros::ADIDigitalOut endgame('G');
  pros::ADIDigitalOut pistonBoost('H');

  pistonBoost.set_value(false);
  // This is preference to what you like to drive on.

  endgame.set_value(endgame_state);

  while (true) {
    pros::lcd::set_text(0, "PM value: " + to_string(pm.get_value()));
    pros::lcd::set_text(1, "endgame state: " + to_string(endgame_state));

    // chassis.tank(); // Tank control
    chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade

    if (done && controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      launching = true;
      done = false;
    }

    if (launching && pm.get_value() <= maxval) {
      launcher.move_velocity(100);
    } else if (launching && pm.get_value() > minval) {
      launcher.move_velocity(0);
      launching = false;
      launched = true;
    }

    if (launched && pm.get_value() >= minval) {
      launcher.move_velocity(100);
    } else if (launched && pm.get_value() < minval) {
      launcher.move_velocity(0);
      launched = false;
      done = true;
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      endgame.set_value(true);
      endgame_state = true;
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      endgame.set_value(false);
      endgame_state = false;
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && done) {
      Intake.move_velocity(-600);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      Intake.move_velocity(600);
    } else {
      Intake.move_velocity(0);
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      pistonBoost.set_value(true);
    }

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!
                                       // Keep this ez::util::DELAY_TIME
  }
}