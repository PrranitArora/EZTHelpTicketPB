#include "subsystems.hpp"
#include "PD.cpp"
enum ARM_STATE {
    STOPPED,
    HOLD,
    RINGSETUP,
    SCORING,
    RESETTING,
    RINGHOLD,
    TIP,
    FULLSCORE,
    DESCORE
};

inline  PD armPD(0.01, 0.01); // Tunable kP and kD values
  
  const ARM_STATE ARM_START_STATE = ARM_STATE::STOPPED;
  
  // state values key:
  // score: 50000
  // setup: 10000
  // fullscore: 65000
  // descore:
  // hold: 20000
  // tip: 78600
  // reset: 0
  
  class ArmStateMachine {
  private:
    ARM_STATE state = ARM_START_STATE;
    pros::Task *updateTask;
    float targetPos = 0;
  
    void update() {
      float currentPos = armRotation.get_position();
      float error = targetPos - currentPos;
  
      float output = armPD.update(error);
  
      switch (this->state) {
      case ARM_STATE::STOPPED:
        arm.move(0);
        arm2.move(0);
        armPD.reset();
        break;
      case ARM_STATE::TIP:
        targetPos = 78600; 
        arm.move(output);  // Use PD to move arm
        arm2.move(output);
        if (fabs(error) < 50.0)
          this->hold(); // Switch to HOLD state
        break;
      case ARM_STATE::FULLSCORE:
        targetPos = 72000; 
        arm.move(output);  // Use PD to move arm
        arm2.move(output);
        if (fabs(error) < 50.0)
          this->hold(); // Switch to HOLD state
        break;
      case ARM_STATE::DESCORE:
        targetPos = 50000; 
        arm.move(output);  // Use PD to move arm
        arm2.move(output);
        if (fabs(error) < 50.0)
          this->hold(); // Switch to HOLD state
        break;
      case ARM_STATE::HOLD:
        arm.brake();
        arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        arm2.brake();
        arm2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        // arm.move(0);
        break;
      case ARM_STATE::RINGSETUP:
        targetPos = 10000; 
        arm.move(output);  // Use PD to move arm
        arm2.move(output);
        if (fabs(error) < 50.0)
          this->hold(); // Switch to HOLD state
        break;
      case ARM_STATE::SCORING:
        targetPos = 50000; 
        arm.move(output);
        arm2.move(output); // Use PD to move arm
        if (fabs(error) < 50.0)
          this->hold(); // Switch to HOLD state
        break;
  
      case ARM_STATE::RINGHOLD:
        targetPos = 20000;
        arm.move(output);
        arm2.move(output); // Use PD to move arm
        if (fabs(error) < 50.0)
          this->stop(); // Switch to STOPPED state
        break;
      case ARM_STATE::RESETTING:
        targetPos = 0; // Reset to home position
        arm.move(output);
        arm2.move(output); // Use PD to move arm
        if (fabs(error) < 50.0)
          this->stop(); // Switch to STOPPED state
        break;
      }
    }
  
  public:
    void initialize() {
      updateTask = new pros::Task([=]() {
        while (true) {
          this->update();
          pros::lcd::print(9, "State: %d", state);
          pros::delay(10);
        }
      });
    }
  
    void shutdown() {
      if (updateTask != nullptr) {
        updateTask->remove(); // Stop the task
        delete updateTask;    // Free memory
        updateTask = nullptr; // Reset pointer
      }
    }
  
    // tip, fullscore, descore
    void stop() { this->state = ARM_STATE::STOPPED; }
  
    void hold() { this->state = ARM_STATE::HOLD; }
  
    void ringSetup() { this->state = ARM_STATE::RINGSETUP; }
  
    void score() { this->state = ARM_STATE::SCORING; }
  
    void reset() { this->state = ARM_STATE::RESETTING; }
  
    void ringhold() { this->state = ARM_STATE::RINGHOLD; }
  
    void tip() { this->state = ARM_STATE::TIP; }
  
    void fullscore() { this->state = ARM_STATE::FULLSCORE; }
  
    void descore() { this->state = ARM_STATE::DESCORE; }
  };