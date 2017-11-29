class PID_Values {

    private float kP, kI, kD;
    private float integral, prevError;
    float setPoint;

    PID_Values(float setKP, float setKI, float setKD) {
        kP = setKP;
        kI = setKI;
        kD = setKD;

    }

    float calculate(float time, float actual) {
        float error = setPoint - actual;
        integral += (error*time);
        float derivative = (error - prevError)/time;
        prevError = error;
        return (kP*error + kI*integral + kD*derivative);
    }

}
