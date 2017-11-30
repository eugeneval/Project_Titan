class Controller {

    PID_Values posZ = new PID_Values(1.6, 0.01, 1);
    PID_Values angleX = new PID_Values(1, 0.01, 1);
    PID_Values angleY = new PID_Values(1, 0.01, 1);
    PID_Values angleZ = new PID_Values(1, 0.01, 1);

    QuadFrame quad;

    Controller(QuadFrame newQuad) {
        quad = newQuad;
    }

    void setAlt(float desiredAlt) {
        posZ.setPoint = desiredAlt;
    }

    void update(float time) {
        float z = posZ.calculate(time, quad.posZ);
        float xAngle = angleX.calculate(time, quad.angleX);
        float yAngle = angleY.calculate(time, quad.angleY);
        float zAngle = angleZ.calculate(time, quad.angleZ);

        quad.motor1.setThrottle(z - xAngle + yAngle + zAngle);
        quad.motor2.setThrottle(z - xAngle - yAngle - zAngle);
        quad.motor3.setThrottle(z + xAngle - yAngle + zAngle);
        quad.motor4.setThrottle(z + xAngle + yAngle - zAngle);
    }


}
