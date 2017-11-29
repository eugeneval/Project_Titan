class Controller {

    private float integralZ, prevErrorZ;
    private float kPZ = 1.6, kIZ = 0.01, kDZ = 1;
    float setZ;

    QuadFrame quad;

    Controller(QuadFrame newQuad) {
        quad = newQuad;
    }

    void setAlt(float desiredAlt) {
        setZ = desiredAlt;
    }

    void update(float time) {
        float correctionZ = posZPID(time);

        changeAllMotors(correctionZ);
    }

    private void changeAllMotors(float correctionZ) {
        quad.motor1.setThrottle(correctionZ);
        quad.motor2.setThrottle(correctionZ);
        quad.motor3.setThrottle(correctionZ);
        quad.motor4.setThrottle(correctionZ);

    }

    private float posZPID(float time) {
        float errorZ = setZ - quad.posZ;
        integralZ += (errorZ*time);
        float derivativeZ = (errorZ - prevErrorZ)/time;
        prevErrorZ = errorZ;
        return (kPZ*errorZ + kIZ*integralZ + kDZ*derivativeZ);
    }

}
