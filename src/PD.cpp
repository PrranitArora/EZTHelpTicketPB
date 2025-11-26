class PD {
    public:
      float kP;
      float kD;
      float prevError;
    
      PD(float kP, float kD) : kP(kP), kD(kD), prevError(0) {}
    
      float update(const float error) {
        // Calculate derivative
        float derivative = error - prevError;
        prevError = error;
    
        // Calculate PD output
        return error * kP + derivative * kD;
      }
    
      void reset() { prevError = 0; }
};