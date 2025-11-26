#include "main.h"

#include "ArmStateMachine.cpp"
#include "autons.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////
// drive motors
int leftBackMtr = -13;
int leftMiddleMtr = -14;
int leftFrontMtr = -1;
int rightBackMtr = 18;
int rightMiddleMtr = 17;
int rightFrontMtr = 10;

// intake motors
int intakeMtr = -9;
int intakeMtr2 = 11;

int armMtr = 20;
int armMtr2 = 19;
int horizontalEncoderPort = 19;
int verticalEncoderPort = -7;  // negative port means reversed
int opticalSensorPort = 16;
int armRotationPort = 19;
int imuPort = 15;
// pneumatics ports
const char matchLoadPiston = 'a';
const char wingPistonRight = 'b';
const char middleGoalPiston = 'c';
const char wingPistonLeft = 'e';
const char ParkPistonLeft = 'd';
const char ParkPistonRight = 'z';
// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {leftBackMtr, leftMiddleMtr, leftFrontMtr},     // Left Chassis Ports (negative port will reverse it!)
    {rightBackMtr, rightMiddleMtr, rightFrontMtr},  // Right Chassis Ports (negative port will reverse it!)

    imuPort,  // IMU Port
    3.25,     // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);     // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels

double calculateMotorOutput(int input, double smoothingFactor) {
  double scaledInput = (double)input / 127.0;  // Scale joystick input to -1 to 1
  return pow(scaledInput, 3) * 127 *
         smoothingFactor;  // Apply cubic scaling and smoothing
}

// Create PD controller for the arm

void Intake(int voltage) {
  intake.move(voltage);
  // intake_second.move(voltage);
}

void Intake2(int voltage) {
  intake2.move(voltage);
  // intake_second.move(voltage);
}

pros::Optical optical(opticalSensorPort);

void colorSortIntakeRed(int voltage, int waitBeforeStop, int stopDuration) {
  static uint32_t startTime = 0;
  static bool waitingToStop = false;
  static bool stopped = false;
  static bool detected = false;  // This latches the detection event

  float hue = optical.get_hue();
  optical.set_led_pwm(100);

  // Detect red and start the sequence if it hasn't already started
  if (!detected && hue < 20) {
    detected = true;
    startTime = pros::millis();
    waitingToStop = true;
  }

  // If we detected red earlier, continue executing the sequence
  if (detected) {
    if (waitingToStop && pros::millis() - startTime >= waitBeforeStop) {
      Intake(-127);  // Move backward after the wait period
      startTime = pros::millis();
      waitingToStop = false;
      stopped = true;
    }

    if (stopped && pros::millis() - startTime >= stopDuration) {
      Intake(voltage);  // Resume normal intake after stop duration
      stopped = false;
      detected = false;  // Reset detection for the next occurrence
    }
  } else {
    Intake(voltage);  // Default behavior when no color detected
  }
}

void colorSortIntakeBlue(int voltage, int waitBeforeStop, int stopDuration) {
  static uint32_t startTime = 0;
  static bool waitingToStop = false;
  static bool stopped = false;
  static bool detected = false;  // This latches the detection event

  float hue = optical.get_hue();
  optical.set_led_pwm(100);

  // Detect red and start the sequence if it hasn't already started
  if (!detected && hue > 190) {
    detected = true;
    startTime = pros::millis();
    waitingToStop = true;
  }

  // If we detected red earlier, continue executing the sequence
  if (detected) {
    if (waitingToStop && pros::millis() - startTime >= waitBeforeStop) {
      Intake(-127);  // Move backward after the wait period
      startTime = pros::millis();
      waitingToStop = false;
      stopped = true;
    }

    if (stopped && pros::millis() - startTime >= stopDuration) {
      Intake(voltage);  // Resume normal intake after stop duration
      stopped = false;
      detected = false;  // Reset detection for the next occurrence
    }
  } else {
    Intake(voltage);  // Default behavior when no color detected
  }
}

void CapacityExtenderTask() {
  while (true) {
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake.move(-127);
      intake2.move(-127);
    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake.move(-127);
      if (capacityExtender.get_distance() > 70) {
        intake2.move(-25);
      } else {
        intake2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      }

      // intake2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.move(127);
      intake2.move_voltage(12000);
    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      // score the middle goal
      MiddleGoal.set_value(true);

      intake.move(-127);
      intake2.move(-70);
    } else {
      MiddleGoal.set_value(false);
      intake.move(0);
      intake2.move(0);
      intake2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

void checkMotorsAndReturnTemperature() {
  std::vector<pros::Motor> motors = {left_back_mtr, left_middle_mtr,
                                     left_front_mtr, right_front_mtr,
                                     right_back_mtr, right_middle_mtr, intake, arm, arm2};

  double totalTemp = 0.0;
  int count = 0;

  for (auto &motor : motors) {
    double temp = motor.get_temperature();
    if (temp ==
        PROS_ERR_F) {  // PROS_ERR_F is returned when the motor is unplugged
      master.set_text(
          0, 0, "Motor " + std::to_string(motor.get_port()) + " unplugged.");
      // pros::delay(250); // A delay of 250 milliseconds before the next
      // function
      master.rumble("---");  // Vex controller vibrates
    }

    if (count < 6) {
      totalTemp += temp;
    }
    ++count;
  }

  if (count == 0)
    master.set_text(0, 0, "No motors found.");

  double averageTempCelsius = totalTemp / count;
  double averageTempFahrenheit = averageTempCelsius * 9.0 / 5.0 + 32.0;
  master.set_text(0, 0, "Avg Temp: " + std::to_string(averageTempFahrenheit));

  // pros::delay(250);
}

void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();
  pros::Task motorCheck(checkMotorsAndReturnTemperature);

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  // ez::as::auton_selector.autons_add({
  //     //{"Drive\n\nDrive forward and come back", drive_example},
  //     {"Turn\n\nTurn 3 times.", PushBackRedLeft},

  // });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

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
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */
  // void skillsLMSD();
  PushBackBlueRight();
  //  PushBackBlueLeftLMSD();
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
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
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  //  set up arm
  // ready controllers
  // position trackers

  intake2.set_voltage_limit(12000);

  bool leftWingExtended = false;
  // WingPistonRight.set_value(false);
  bool rightWingExtended = false;
  bool matchLoaderExtended = false;
  bool doubleParkExtended = false;
  bool chopSticksExtended = false;
  int defaultForwardDirection = 1;
  MiddleGoal.set_value(true);
  MatchLoader.set_value(false);

  bool readyRightWing = true;
  bool readyMatchLoader = true;

  // op control based on signal controller setup
  double start_time = pros::millis();
  bool sec_30 = false;
  bool sec_15 = false;

  bool loaded = false;
  pros::Task capacityExtenderTask(CapacityExtenderTask);
  // tuning PID
  // chassis.pid_tuner_enable();

  while (true) {
    // tuning pid

    // chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate

    // Gives you some extras to make EZ-Template ezier

    // chassis.opcontrol_tank();  // Tank control
    chassis.opcontrol_arcade_standard(ez::SPLIT);  // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      // trigger match loader
      if (!matchLoaderExtended) {
        MatchLoader.set_value(true);

        matchLoaderExtended = true;
      } else if (matchLoaderExtended) {
        MatchLoader.set_value(false);

        matchLoaderExtended = false;
      }
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      // trigger right wing
      if (!rightWingExtended) {
        WingPistonRight.set_value(true);
        pros::delay(50);
        rightWingExtended = true;
      } else {
        WingPistonRight.set_value(false);
        pros::delay(50);
        rightWingExtended = false;
      }
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      // trigger right wing
      if (!doubleParkExtended) {
        DoublePark.set_value(true);
        pros::delay(50);
        doubleParkExtended = true;
      } else {
        DoublePark.set_value(false);
        pros::delay(50);
        doubleParkExtended = false;
      }
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      // trigger right wing
      if (!chopSticksExtended) {
        ChopSticks.set_value(true);
        pros::delay(50);
        chopSticksExtended = true;
      } else {
        ChopSticks.set_value(false);
        pros::delay(50);
        chopSticksExtended = false;
      }
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
