class PID_Values {

    private float kP, kI, kD, integralWindUp = 250;
    private float integral, prevError, maxSetPoint;
    float setPoint = 0;
    private boolean existsMaxSetPoint = false, isPositionController = false;

    PID_Values(float setKP, float setKI, float setKD) {
        kP = setKP;
        kI = setKI;
        kD = setKD;
        existsMaxSetPoint = false;
    }

    PID_Values(float setKP, float setKI, float setKD, float setMaxSetPoint) {
        kP = setKP;
        kI = setKI;
        kD = setKD;
        maxSetPoint = setMaxSetPoint;
        existsMaxSetPoint = true;
    }
    PID_Values(float setKP, float setKI, float setKD, boolean setIsPositionController) {
        kP = setKP;
        kI = setKI;
        kD = setKD;
        isPositionController = setIsPositionController;
    }

    void set(float newSetPoint) {
        if (existsMaxSetPoint) {
            if (newSetPoint > maxSetPoint) {
                newSetPoint = maxSetPoint;
            }
            else if (newSetPoint < -maxSetPoint) {
                newSetPoint = -maxSetPoint;
            }
        }
        setPoint = newSetPoint;
    }

    float calculate(float time, float actual) {
        float error = setPoint - actual;
        integral += (error*time);
        if (integral > integralWindUp) {
            integral = 0;
        }
        float derivative = (error - prevError)/time;
        prevError = error;
        if (isPositionController) {
            return (-kP*error + kI*integral + kD*derivative);
        } else {
            return (kP*error + kI*integral + kD*derivative);
        }
    }

}
