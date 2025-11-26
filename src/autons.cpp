#include "ArmStateMachine.cpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int TURTLESPEED = 45;
const int SLOWDRIVESPEED = 70;
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;
const int INTAKE = -127;
const int OUTTAKE = 127;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20, 2.0, 110);             // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(4.0, 0.1, 20.0, 15.0);      // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  // chassis.pid_wait();

  // chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  // chassis.pid_wait();
}

// tested!

void PushBackRedLeft() {
  chassis.pid_drive_set(15_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-16_deg, TURN_SPEED);
  chassis.pid_wait();
  // start intaking at max speed
  intake.move(INTAKE);
  // drive forward to first cluster of balls
  chassis.pid_drive_set(26_in, TURTLESPEED);
  chassis.pid_wait();
  // turn to balls under goal
  chassis.pid_turn_set(28_deg - 90_deg, TURN_SPEED);  // -12 + 40
  chassis.pid_wait();
  // drive to balls under goal
  chassis.pid_drive_set(24_in, TURTLESPEED + 10);
  chassis.pid_wait();
  // drive back to get clearance to line up with goal
  chassis.pid_drive_set(-14_in, DRIVE_SPEED);
  chassis.pid_wait();
  // turn to axis into goal
  chassis.pid_turn_set(291_deg - 90_deg, TURN_SPEED);  // 251 + 40
  chassis.pid_wait();
  // move into goal axis
  chassis.pid_drive_set(36_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(150);
  // turn to the goal
  chassis.pid_turn_set(258.5_deg - 90_deg, TURN_SPEED);  // 218.5 + 40
  chassis.pid_wait();
  // drive to goal (backwards)
  chassis.pid_drive_set(-19.5_in, DRIVE_SPEED);
  chassis.pid_wait();
  // move second stage intake to score balls
  intake2.move(INTAKE);
  // drive into goal to maintain position
  chassis.pid_drive_set(-1.5_in, DRIVE_SPEED);
  // finish scoring
  pros::delay(900);
  chassis.pid_wait();
  // stop second stage intake
  intake2.move(0);
  // extend match loader
  MatchLoader.extend();
  // move into match load
  chassis.pid_turn_set(256_deg - 90_deg + 10_deg, TURN_SPEED);  // 216 + 40
  chassis.pid_wait();
  chassis.pid_drive_set(31_in, DRIVE_SPEED - 16);
  chassis.pid_wait();
  chassis.pid_drive_set(1.5_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(700);
  chassis.pid_turn_set(256_deg - 90_deg + 10_deg, TURN_SPEED);  // 216 + 40
  chassis.pid_wait();
  // move back to goal
  chassis.pid_drive_set(-27_in, DRIVE_SPEED);
  chassis.pid_wait();
  // score collected balls (remaining 3)
  intake2.move(INTAKE);
}

// untested

void PushBackBlueLeft() {
  chassis.pid_drive_set(15_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-16_deg, TURN_SPEED);
  chassis.pid_wait();
  // start intaking at max speed
  intake.move(INTAKE);
  // drive forward to first cluster of balls
  chassis.pid_drive_set(26_in, TURTLESPEED);
  chassis.pid_wait();
  // turn to balls under goal
  chassis.pid_turn_set(28_deg - 90_deg + 6_deg, TURN_SPEED);  // -12 + 40
  chassis.pid_wait();
  // drive to balls under goal
  chassis.pid_drive_set(22_in, TURTLESPEED + 10);
  chassis.pid_wait();
  // drive back to get clearance to line up with goal
  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
  // turn to axis into goal
  chassis.pid_turn_set(291_deg - 90_deg, TURN_SPEED);  // 251 + 40
  chassis.pid_wait();
  // move into goal axis
  chassis.pid_drive_set(36_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(150);
  // turn to the goal
  chassis.pid_turn_set(258.5_deg - 90_deg + 4_deg, TURN_SPEED);  // 218.5 + 40
  chassis.pid_wait();
  // drive to goal (backwards)
  chassis.pid_drive_set(-19.5_in - 3.5_in, DRIVE_SPEED);
  chassis.pid_wait();
  // move second stage intake to score balls
  intake2.move(INTAKE);
  // drive into goal to maintain position
  chassis.pid_drive_set(-1.5_in, DRIVE_SPEED);
  // finish scoring
  pros::delay(600);
  chassis.pid_wait();
  // stop second stage intake
  intake2.move(0);
  // extend match loader
  MatchLoader.extend();
  // move into match load
  chassis.pid_turn_set(256_deg - 90_deg + 10_deg + 4_deg, TURN_SPEED);  // 216 + 40
  chassis.pid_wait();
  chassis.pid_drive_set(31_in, DRIVE_SPEED - 15);
  chassis.pid_wait();
  chassis.pid_drive_set(0.75_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_turn_set(256_deg - 90_deg + 10_deg, TURN_SPEED);  // 216 + 40
  chassis.pid_wait();
  // move back to goal
  chassis.pid_drive_set(-27_in, DRIVE_SPEED);
  chassis.pid_wait();
  // score collected balls (remaining 3)
  intake2.move(INTAKE);
}
// untested
void PushBackRedRight() {
  chassis.pid_drive_set(15_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(360_deg - (-16_deg), TURN_SPEED);  // = 376 -> normalized to 16_deg
  chassis.pid_wait();
  // start intaking at max speed
  intake.move(INTAKE);
  // drive forward to first cluster of balls
  chassis.pid_drive_set(26_in, TURTLESPEED);
  chassis.pid_wait();
  // turn to balls under goal (mirror of 28 - 90 = -62°)
  chassis.pid_turn_set(360_deg - (28_deg - 90_deg), TURN_SPEED);  // = 422 -> normalized to 62_deg
  chassis.pid_wait();
  // drive to balls under goal
  chassis.pid_drive_set(24_in, TURTLESPEED + 10);
  chassis.pid_wait();
  // drive back to get clearance to line up with goal
  chassis.pid_drive_set(-14_in, DRIVE_SPEED);
  chassis.pid_wait();
  // turn to axis into goal (mirror of 291 - 90 = 201°)
  chassis.pid_turn_set(360_deg - (291_deg - 90_deg), TURN_SPEED);  // = 159_deg
  chassis.pid_wait();
  // move into goal axis
  chassis.pid_drive_set(36_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(150);
  // turn to the goal (mirror of 258.5 - 90 = 168.5°)
  chassis.pid_turn_set(360_deg - (258.5_deg - 90_deg), TURN_SPEED);  // = 191.5_deg
  chassis.pid_wait();
  // drive to goal (backwards)
  chassis.pid_drive_set(-19.5_in, DRIVE_SPEED);
  chassis.pid_wait();
  // move second stage intake to score balls
  intake2.move(INTAKE);
  // drive into goal to maintain position
  chassis.pid_drive_set(-1.5_in, DRIVE_SPEED);
  // finish scoring
  pros::delay(900);
  chassis.pid_wait();
  // stop second stage intake
  intake2.move(0);
  // extend match loader
  MatchLoader.extend();
  // move into match load (mirror of 256 - 90 + 10 = 176°)
  chassis.pid_turn_set(360_deg - (256_deg - 90_deg + 10_deg), TURN_SPEED);  // = 184_deg
  chassis.pid_wait();
  chassis.pid_drive_set(31_in, DRIVE_SPEED - 16);
  chassis.pid_wait();
  chassis.pid_drive_set(1.5_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(700);
  chassis.pid_turn_set(360_deg - (256_deg - 90_deg + 10_deg), TURN_SPEED);  // = 184_deg
  chassis.pid_wait();
  // move back to goal
  chassis.pid_drive_set(-27_in, DRIVE_SPEED);
  chassis.pid_wait();
  // score collected balls (remaining 3)
  intake2.move(INTAKE);
}

// untested
void PushBackBlueRight() {
  MiddleGoal.retract();
  chassis.pid_drive_set(15_in, DRIVE_SPEED);
  intake.move_voltage(-12000);
  chassis.pid_wait();
  chassis.pid_turn_set(362_deg - (-16_deg), TURN_SPEED);  // = 376° → 16°
  chassis.pid_wait();
  // start intaking at max speed
  intake.move_voltage(-12000);
  // drive forward to first cluster of balls
  chassis.pid_drive_set(25_in, TURTLESPEED);
  chassis.pid_wait();
  intake.move_voltage(-12000);
  // turn to balls under goal (mirror of 28 - 90 = -62°)
  chassis.pid_turn_set(356_deg - (28_deg - 90_deg), TURN_SPEED);  // = 422° → 62°
  chassis.pid_wait();
  intake.move_voltage(-12000);
  // drive to balls under goal
  chassis.pid_drive_set(22_in, TURTLESPEED + 10);
  chassis.pid_wait();
  intake.move_voltage(-12000);
  // drive back to get clearance to line up with goal
  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
  intake.move_voltage(-12000);
  // turn to axis into goal (mirror of 291 - 90 = 201°)
  chassis.pid_turn_set(362_deg - (291_deg - 90_deg), TURN_SPEED);  // = 159°
  chassis.pid_wait();
  intake.move_voltage(-12000);
  // move into goal axis
  chassis.pid_drive_set(36_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(150);
  // turn to the goal (mirror of 258.5 - 90 = 168.5°)
  chassis.pid_turn_set(358_deg - (258.5_deg - 90_deg), TURN_SPEED);  // = 191.5°
  chassis.pid_wait();
  // drive to goal (backwards)
  chassis.pid_drive_set(-21.5_in, DRIVE_SPEED);
  chassis.pid_wait();
  // move second stage intake to score balls
  intake2.move_voltage(-12000);
  // drive into goal to maintain position
  chassis.pid_drive_set(-1.5_in, DRIVE_SPEED);
  // finish scoring
  pros::delay(600);
  chassis.pid_wait();
  // stop second stage intake
  intake2.move(0);
  // extend match loader
  MatchLoader.extend();
  // move into match load (mirror of 256 - 90 + 10 = 176°)
  chassis.pid_turn_set(360_deg - (256_deg - 90_deg + 14_deg), TURN_SPEED);  // = 184°
  chassis.pid_wait();
  chassis.pid_drive_set(32.5_in, DRIVE_SPEED - 25);
  chassis.pid_wait();
  // chassis.pid_drive_set(0.75_in, DRIVE_SPEED);
  // chassis.pid_wait();
  pros::delay(500);
  chassis.pid_turn_set(360_deg - (256_deg - 90_deg + 14_deg), TURN_SPEED);  // = 184°
  chassis.pid_wait();
  // move back to goal
  chassis.pid_drive_set(-31.5_in, DRIVE_SPEED);
  chassis.pid_wait();
  // score collected balls (remaining 3)
  intake2.move_voltage(-12000);
}

void PushBackBlueRightLMSD() {
  chassis.pid_drive_set(-34, 127, true);
  chassis.pid_wait();

  // 90 → 360 - 90 = 270
  chassis.pid_turn_set(270.0, 127);
  chassis.pid_wait();

  MatchLoader.extend();
  intake.move(INTAKE);
  pros::delay(50);

  chassis.pid_drive_set(16.5, 75, true);
  chassis.pid_wait();
  pros::delay(550);

  // 89 → 360 - 89 = 271
  chassis.pid_turn_set(271.0, 127);
  chassis.pid_wait();

  chassis.pid_drive_set(-33.0, 127, true);
  chassis.pid_wait();

  MatchLoader.retract();
  intake.move(INTAKE);
  pros::delay(50);
  intake2.move(INTAKE);
  pros::delay(50);
  pros::delay(1250);

  chassis.pid_drive_set(12.0, 127, true);
  chassis.pid_wait();

  intake2.move(0);

  // (149 + 180) = 329 → 360 - 329 = 31
  chassis.pid_turn_set(31.0, 127);
  chassis.pid_wait();

  intake.move(INTAKE);
  pros::delay(50);

  chassis.pid_drive_set(35.5, 25);
  intake.move(INTAKE);

  chassis.pid_wait_until(26_in);
  intake.move(INTAKE);

  MatchLoader.extend();
  intake.move(INTAKE);
  chassis.pid_wait();
  intake.move(INTAKE);
  MatchLoader.retract();
  intake.move(INTAKE);
  // (125 + 7) = 132 → 360 - 132 = 228
  chassis.pid_turn_set(228.0, 127);
  intake.move(INTAKE);
  chassis.pid_wait();
  intake.move(INTAKE);
  chassis.pid_drive_set(-18, 127, true);
  intake.move(INTAKE);
  chassis.pid_wait();
  intake.move(INTAKE);
  MiddleGoal.extend();
  intake.move(-127);
  pros::delay(50);
  intake2.move(-70);
  pros::delay(50);
  pros::delay(250);

  // ------------------------
  // Reflected but commented code (kept commented)
  // ------------------------

  // (128 + 8) = 136 → 360 - 136 = 224
  // chassis.pid_turn_set(224, 127);
  // chassis.pid_wait();

  // chassis.pid_drive_set(8.0, 127, true);
  // chassis.pid_wait();

  // 180 → 360 - 180 = 180
  // chassis.pid_turn_set(180, 127);
  // chassis.pid_wait();

  // chassis.pid_drive_set(-39.0, 127, true);
  // chassis.pid_wait();

  // (45 + 180) = 225 → 360 - 225 = 135
  // chassis.pid_turn_set(135.0, 127);
  // chassis.pid_wait();

  // chassis.pid_drive_set(9.0, 127, true);
  // chassis.pid_wait();

  // intake.move(127);
  // pros::delay(100);
  // intake2.move(127);
  // pros::delay(100);

  //----------------------------------

  // -47.5 → 360 - (-47.5) = 407.5
  // chassis.pid_turn_set(407.5, 127);
  // chassis.pid_wait();

  // chassis.pid_drive_set(5.5, 127, true);
  // chassis.pid_wait();

  // 135.5 → 360 - 135.5 = 224.5
  // chassis.pid_turn_set(224.5, 127);
  // chassis.pid_wait();

  // chassis.pid_drive_set(25.5, 127, true);
  // chassis.pid_wait();

  // 112 → 360 - 112 = 248
  // chassis.pid_turn_set(248.0, 127);
  // chassis.pid_wait();

  // chassis.pid_drive_set(16.5, 127, true);
  // chassis.pid_wait();

  // 0 → 360 - 0 = 360
  // chassis.pid_turn_set(360.0, 127);
  // chassis.pid_wait();
}

void PushBackBlueLeftLMSD() {
  MiddleGoal.retract();
  chassis.pid_drive_set(-32.5, 127, true);
  intake.move_voltage(-12000);
  pros::delay(100);
  chassis.pid_wait();
  intake.move_voltage(-12000);
  chassis.pid_turn_set(90.0, 127);
  intake.move_voltage(-12000);
  chassis.pid_wait();
  intake.move_voltage(-12000);
  MatchLoader.extend();
  intake.move_voltage(-12000);
  pros::delay(50);
  intake.move_voltage(-12000);
  chassis.pid_drive_set(16.5, 85, true);
  intake.move_voltage(-12000);
  chassis.pid_wait();
  intake.move_voltage(-12000);

  chassis.pid_drive_set(-1, 85, true);
  chassis.pid_wait();
  chassis.pid_drive_set(1, 85, true);
  chassis.pid_wait();
  // finish match loading
  intake.move_voltage(-12000);
  chassis.pid_turn_set(90.5, 127);
  intake.move_voltage(-12000);
  chassis.pid_wait();
  intake.move_voltage(-12000);
  chassis.pid_drive_set(-34.0, 127, true);
  intake.move_voltage(-12000);
  chassis.pid_wait();
  intake2.move_voltage(-12000);
  MatchLoader.retract();
  intake2.move_voltage(-12000);
  pros::delay(50);
  intake2.move_voltage(-12000);
  pros::delay(50);
  intake.move(INTAKE);
  pros::delay(800);
  intake.move(INTAKE);
  chassis.pid_drive_set(12.0, 127, true);
  intake.move(INTAKE);
  chassis.pid_wait();
  intake.move(INTAKE);
  intake2.move(0);
  intake.move(INTAKE);
  chassis.pid_turn_set(149 + 180, 127);
  intake.move(INTAKE);
  chassis.pid_wait();
  intake.move(INTAKE);
  pros::delay(50);
  intake.move(INTAKE);
  chassis.pid_drive_set(34.5, 25);
  intake.move(INTAKE);

  chassis.pid_wait_until(26_in);
  intake.move(INTAKE);

  MatchLoader.extend();

  chassis.pid_wait();
  MatchLoader.retract();
  chassis.pid_turn_set(125 + 7, 127);
  chassis.pid_wait();
  chassis.pid_drive_set(-12, 127, true);
  chassis.pid_wait();
  MiddleGoal.extend();
  intake.move(-127);
  pros::delay(50);
  intake2.move(-70);
  pros::delay(50);
  pros::delay(250);

  // chassis.pid_turn_set(128 + 8, 127);
  // chassis.pid_wait();
  // chassis.pid_drive_set(8.0, 127, true);
  // chassis.pid_wait();
  // chassis.pid_turn_set(180, 127);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-39.0, 127, true);
  // chassis.pid_wait();
  // chassis.pid_turn_set(45.0 + 180, 127);
  // chassis.pid_wait();
  // chassis.pid_drive_set(9.0, 127, true);
  // chassis.pid_wait();
  // intake.move(127);
  // pros::delay(100);
  // intake2.move(127);
  // pros::delay(100);
  //----------------------------------
  // chassis.pid_turn_set(-47.5, 127);
  // chassis.pid_wait();
  // chassis.pid_drive_set(5.5, 127, true);
  // chassis.pid_wait();
  // chassis.pid_turn_set(135.5, 127);
  // chassis.pid_wait();
  // chassis.pid_drive_set(25.5, 127, true);
  // chassis.pid_wait();
  // chassis.pid_turn_set(112.0, 127);
  // chassis.pid_wait();
  // chassis.pid_drive_set(16.5, 127, true);
  // chassis.pid_wait();
  // chassis.pid_turn_set(0.0, 127);
  // chassis.pid_wait();
}

void skillsLMSD() {
  DoublePark.extend();
  pros::delay(50000);
}

void skillsStates() {
  ArmStateMachine autonArm{};
  autonArm.initialize();
  armRotation.set_position(10000);
  pros::delay(400);
  autonArm.fullscore();
  pros::delay(500);
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-29_in, DRIVE_SPEED / 2.5);
  chassis.pid_wait();
  autonArm.reset();
  pros::delay(350);
  chassis.pid_wait();
  mogo_mech.extend();
  pros::delay(250);
  chassis.pid_wait();
  autonArm.reset();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  intake.move(127);
  chassis.pid_drive_set(22_in, DRIVE_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  chassis.pid_turn_set(210_deg, TURN_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  chassis.pid_drive_set(43_in, DRIVE_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  chassis.pid_drive_set(32_in, DRIVE_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  autonArm.reset();
  chassis.pid_turn_set(210_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(450);
  autonArm.ringSetup();
  pros::delay(250);

  chassis.pid_drive_set(28_in, DRIVE_SPEED / 1.5);

  autonArm.ringSetup();
  chassis.pid_wait();

  chassis.pid_drive_set(-28_in, DRIVE_SPEED / 1.5);
  chassis.pid_wait();

  autonArm.ringSetup();

  chassis.pid_turn_set(180_deg, TURN_SPEED / 1.5);
  chassis.pid_wait();
  autonArm.ringSetup();
  intake.move(0);
  pros::delay(150);
  intake.move(127);
  pros::delay(100);
  intake.move(0);
  pros::delay(120);
  intake.move(127);
  pros::delay(100);
  intake.move(0);
  chassis.pid_drive_set(-35_in, DRIVE_SPEED / 1.5);
  chassis.pid_wait();
  autonArm.ringSetup();
  chassis.pid_turn_set(260_deg, TURN_SPEED / 1.5);
  chassis.pid_wait();
  pros::delay(250);
  intake.move(0);
  pros::delay(50);
  autonArm.ringhold();
  pros::delay(350);
  intake.move(127);
  pros::delay(150);
  chassis.pid_drive_set(32_in, DRIVE_SPEED / 1.5);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, DRIVE_SPEED / 1.2);
  autonArm.score();
  pros::delay(800);
  intake.move(127);
  chassis.pid_wait();
  chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  chassis.pid_drive_set(79_in, DRIVE_SPEED / 2);
  chassis.pid_wait();
  autonArm.reset();
  chassis.pid_turn_set(210_deg, TURN_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  chassis.pid_drive_set(15_in, DRIVE_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  chassis.pid_drive_set(-18_in, DRIVE_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  mogo_mech.retract();
  pros::delay(350);
  intake.move(-127);
  pros::delay(150);
  chassis.pid_drive_set(5_in, DRIVE_SPEED);
  chassis.pid_wait();
  autonArm.reset();
}

void bakerRed() {
  ArmStateMachine autonArm{};
  autonArm.initialize();
  armRotation.set_position(10000);
  pros::delay(400);
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(5_in, DRIVE_SPEED);
  chassis.pid_wait();
  autonArm.fullscore();
  pros::delay(700);
  chassis.pid_drive_set(-8_in, DRIVE_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  pros::delay(800);
  chassis.pid_turn_set(15_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-34_in, 50);
  chassis.pid_wait();
  autonArm.reset();
  mogo_mech.extend();
  pros::delay(300);
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  chassis.pid_drive_set(24_in, 90);
  chassis.pid_wait();
  sweeperRight.extend();
  autonArm.reset();
  chassis.pid_drive_set(-27_in, DRIVE_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  sweeperRight.retract();
  chassis.pid_turn_set(155_deg, TURN_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  intake.move(127);
  chassis.pid_drive_set(15_in, 90);
  chassis.pid_wait();
  autonArm.reset();
  chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  chassis.pid_wait();
  autonArm.reset();
  pros::delay(1000);
  chassis.pid_turn_set(270, TURN_SPEED);
}
///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();

  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    // chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .