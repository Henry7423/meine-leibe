/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Le Mien                                          */
/*    Created:      Thurs January 1st, 1970                                           */
/*    Description:  IP logger                                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

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
motor Motor12 = motor(PORT12, ratio6_1, false);



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
#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/motors.h"


 /**
	* Motor Declarations
 */
pros::MotorGroup left_motor_group({ 1, 2, 3 }, pros::MotorGearset::blue); // Numbers are port numbers, negative is reversed
pros::MotorGroup right_motor_group({ 4, 5, 6 }, pros::MotorGearset::blue); // Numbers are port numbers, negative is reversed



/**
	* Drivetrain Settings
*/
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
							  &right_motor_group, // right motor group
							  11.25, // 11.25 inch track width
							  lemlib::Omniwheel::NEW_325, // 2.75 inch wheel diameter
							  450, // 450 rpm drive speed
							  2 // horizontal drive (CHANGE LATER)
							  );

/**
	* IMU and Tracking Declarations

	* Can use either rotation sensor or paired encoder ports ('A', 'B') - Check LemLib Documentation fore more info
*/
pros::Imu imu(5); // Port 5
pros::Rotation vertical_sensor(15); // Port 15 and Reversed
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, lemlib::Omniwheel::NEW_275, -0.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
							nullptr, // vertical tracking wheel 2, that does not exist - set to null
							nullptr, // horizontal tracking wheel 1, that does not exist - set to null
							nullptr, // horizontal tracking wheel 2, that does not exist - set to null
							&imu // inertial sensor
);
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}


/**
	* PID Constants
*/

// lateral PID controller
lemlib::ControllerSettings lateral_controller(6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              4.5, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(4.75, // proportional gain (kP)
                                              0, // integral gain (kI) - not used
                                              3.5, // derivative gain (kD)
                                              0, // anti windup - not used
                                              3, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              15, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew) - not used
);

/**
	* Chassis Constructor
*/

lemlib::Chassis chassis(drivetrain, // drivetrain settings
						lateral_controller, // lateral PID settings
						angular_controller, // angular PID settings
						sensors // odometry settings
					    );

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
	// set pos to x: 0, y: 0, theta: 0
	chassis.setPose(0,0,0);
	// turn to face 90 with very long timeout
	chassis.turnToHeading(-90, 4000);
	chassis.moveToPoint(-24,0, 4000);
	chassis.turnToHeading(0, 4000);
	chassis.moveToPoint(-24, 24, 4000);
	chassis.turnToHeading(90, 4000);
	chassis.moveToPoint(0, 24, 4000);
	chassis.turnToHeading(180, 4000);
	chassis.moveToPoint(0, 0, 4000);
	chassis.turnToHeading(0, 4000);

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


pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Button Declarations
bool intakeButton = false;
bool outtakeButton = false;

// PTO Declarations
bool PTOButton = false;

// Mogo Declarations
bool mogoToggle = false;

// Macro Declarations
bool armUp = false;

bool armDown = false;


void opcontrol() {
    // loop forever
    while (true) {
		/* Button Assignments */

		intakeButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
		outtakeButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
		mogoToggle = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);

		if (intakeButton) {
			// intake
      Motor12.spin(forward);

		}
		if (outtakeButton) {
			// outtake
      Motor12.spin(reverse);
		}

		if (mogoToggle) {
			// do something
		}

		/* Controller Movements */
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(rightX, leftY, false, 0.5);

        // delay to save resources
        pros::delay(25);

    }
}