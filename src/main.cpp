#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor l1_front(19, true);
pros::Motor l2_front(20);
pros::Motor r1_front(12, true);
pros::Motor r2_front(14);
pros::Motor l1_back(17, true);
pros::Motor l2_back(18);
pros::Motor r1_back(15, true);
pros::Motor r2_back(16);

pros::Motor flywheel (4, MOTOR_GEARSET_06, false, MOTOR_ENCODER_DEGREES);
pros::Motor intake(3);

void set_drive(int fl, int bl, int fr, int br) {
  l1_front = fl;
  l2_front = fl;
  l1_back = bl;
  l2_back = bl;
  r1_front = fr;
  r2_front = fr;
  r1_back = br;
  r2_back = br;
}

void set_flywheel(int input) {
  flywheel = input;
}

void set_intake(int input) {
  intake = input;
}



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
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
void autonomous() {}

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

double targetRPM = 0;
double getRPM() {
  return flywheel.get_actual_velocity()*5;
}
bool flywheel_on = false;
void flywheel_control() {
     const double kP = 3.0;

     const double maxVoltage = 12000;

     // Geared for 3000RPM (600 5:1), but this is the measured max RPM
     const double maxRPM = 3200;

     // Multiplied by a scalar to adjust for the real world
     const double kv = 0.98 * maxVoltage / maxRPM;

     // "Acceptable" velocity range to switch to hold power + p control
     double deadband = 100;

     while (1) {
         //rpmGuard.take(2);


         double rpmError = targetRPM - getRPM();

         double holdPower = targetRPM * kv;

         double output = 0;

         // Coast down
         if (targetRPM == 0) {
           pros::lcd::set_text(2, "Flywheel Off!         ");
             output = 0;
           }
         // Running too slow (< target - deadband), apply full power
         else if (rpmError > deadband) {
           pros::lcd::set_text(2, "Flywheel Too Slow!      ");
             output = maxVoltage;
           }
         // Running too fast (> target + deadband), apply half of hold power
         else if (rpmError < -deadband) {
           pros::lcd::set_text(2, "Flywheel Too Fast!      ");
             output = holdPower * 0.5;
           }
         // Running within deadband, switch to hold power + p contorl
         else {
             output = holdPower + kP * rpmError;
             pros::lcd::set_text(2, "At Velocity!          ");
           }

         if (targetRPM >= 0) {
             //flywheelMotor.moveVoltage(output);
             //set_flywheel(output);
             flywheel.move_voltage(output);
             //lastCommandedVoltage = 0;
         }

         //rpmGuard.give();
       //delay(sampleTime);
       std::string str_rpm = std::to_string(targetRPM);
       std::string str_cur = std::to_string((int)getRPM());
       pros::lcd::set_text(0, "Target Vel: " + str_rpm);
       pros::lcd::set_text(1, "Current Vel: " + str_cur);
       pros::delay(10);
     }
 }
 pros::Task flywheelControl(flywheel_control);


void opcontrol() {

	while (true) {

    int Ch3 = master.get_analog(ANALOG_LEFT_Y);
    int Ch4 = master.get_analog(ANALOG_LEFT_X);
    int Ch1 = master.get_analog(ANALOG_RIGHT_X);

    int FrontLeft = Ch3 + Ch1 + Ch4;
    int RearLeft = Ch3 + Ch1 - Ch4;
    int FrontRight = Ch3 - Ch1 - Ch4;
    int RearRight = Ch3 - Ch1 + Ch4;

    set_drive(FrontLeft, RearLeft, FrontRight, RearRight);



    if (master.get_digital_new_press(DIGITAL_L1)) {
      targetRPM = targetRPM != 0 ? 0 : 3230;
    }
    else if (master.get_digital_new_press(DIGITAL_L2)) {
      targetRPM = targetRPM != 0 ? 0 : 3000;
    } else if (master.get_digital_new_press(DIGITAL_UP)) {
       targetRPM = targetRPM != 0 ? 0 : 2800;
     }


    if (master.get_digital(DIGITAL_R1)) {
      set_intake(127);
    } else if (master.get_digital(DIGITAL_R2)) {
      set_intake(-127);
    } else {
      set_intake(0);
    }


		pros::delay(10);
	}
}
