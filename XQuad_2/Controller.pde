class Controller {

    PID_Values posX = new PID_Values(0.1, 0.01, 10, true);
    PID_Values posY = new PID_Values(0.1, 0.01, 10, true);
    PID_Values posZ = new PID_Values(150, 0.03, 100);
    PID_Values angleX = new PID_Values(3, 0.01, 1, radians(25));
    PID_Values angleY = new PID_Values(3, 0.01, 1, radians(25));
    PID_Values angleZ = new PID_Values(3, 0.01, 1);

    QuadFrame quad;

    Controller(QuadFrame newQuad) {
        quad = newQuad;
    }

    void setAlt(float desiredAlt) {
        posZ.set(desiredAlt);
    }

    void update(float time) {
        angleX.set(posX.calculate(time, quad.posX));
        angleY.set(posY.calculate(time, quad.posY));
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
