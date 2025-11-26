#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "main.h"
#include "pros/distance.hpp"
#include "pros/motors.h"
#include "subsystems.hpp"

extern Drive chassis;

inline pros::Motor left_back_mtr(-6, pros::v5::MotorGears::blue,
                                 pros::v5::MotorEncoderUnits::degrees);
inline pros::Motor left_middle_mtr(-3, pros::v5::MotorGears::blue,
                                   pros::v5::MotorEncoderUnits::degrees);
inline pros::Motor left_front_mtr(-4, pros::v5::MotorGears::blue,
                                  pros::v5::MotorEncoderUnits::degrees);

inline pros::Motor right_back_mtr(10, pros::v5::MotorGears::blue,
                                  pros::v5::MotorEncoderUnits::degrees);
inline pros::Motor right_middle_mtr(9, pros::v5::MotorGears::blue,
                                    pros::v5::MotorEncoderUnits::degrees);
inline pros::Motor right_front_mtr(7, pros::v5::MotorGears::blue,
                                   pros::v5::MotorEncoderUnits::degrees);
// motor groups
inline pros::MotorGroup leftMotors(
    {-6, -3, -4},
    pros::MotorGearset::blue);  // left motor group - ports 3 (reversed), 4
                                // (reversed), 5 (reversed)
inline pros::MotorGroup
    rightMotors({10, 9, 7},
                pros::MotorGearset::blue);  // right motor group - ports 6, 7, 9

// pistons"
inline pros::adi::Pneumatics mogo_mech('z', false);
inline pros::adi::Pneumatics sweeperRight('z', false);
inline pros::adi::Pneumatics sweeperLeft('z', false);
inline pros::adi::Pneumatics intakeLift('z', false);

inline pros::adi::Pneumatics WingPistonRight('b', true);
inline pros::adi::Pneumatics MatchLoader('a', false);
inline pros::adi::Pneumatics MiddleGoal('c', true);
inline pros::adi::Pneumatics DoublePark('d', false);
inline pros::adi::Pneumatics ChopSticks('h', false);
////Misc motors////
inline pros::Motor intake(-9, pros::v5::MotorGears::blue,
                          pros::v5::MotorEncoderUnits::degrees);
inline pros::Motor intake2(11, pros::v5::MotorGears::blue,
                           pros::v5::MotorEncoderUnits::degrees);

inline pros::Motor arm(20, pros::v5::MotorGears::green,
                       pros::v5::MotorEncoderUnits::degrees);
inline pros::Motor arm2(20, pros::v5::MotorGears::green,
                        pros::v5::MotorEncoderUnits::degrees);

inline pros::Rotation armRotation(20);
inline pros::Distance capacityExtender(8);
