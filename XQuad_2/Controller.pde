class Controller {

    PID_Values posZ = new PID_Values(1.6, 0.01, 1);

    float setX, setY, setZ;
    float setXAngle, setYAngle, setZAngle;

    QuadFrame quad;

    Controller(QuadFrame newQuad) {
        quad = newQuad;
    }

    void setAlt(float desiredAlt) {
        posZ.setPoint = desiredAlt;
    }

    void update(float time) {
        float correctionZ = posZ.calculate(time, quad.posZ);

        changeAllMotors(correctionZ);
    }

    private void changeAllMotors(float correctionZ) {
        quad.motor1.setThrottle(correctionZ);
        quad.motor2.setThrottle(correctionZ);
        quad.motor3.setThrottle(correctionZ);
        quad.motor4.setThrottle(correctionZ);

    }

}
