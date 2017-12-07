class Controller {

    PID_Values posX = new PID_Values(1.7, 0.01, 3.5);
    PID_Values posY = new PID_Values(1.7, 0.01, 3.5);
    PID_Values posZ = new PID_Values(5, 0.05, 3);
    PID_Values angleX = new PID_Values(50, 0.01, 10, radians(25));
    PID_Values angleY = new PID_Values(50, 0.01, 10, radians(25));
    PID_Values angleZ = new PID_Values(50, 0.01, 10);

    final float accuracy = 0.1;

    final float[] wpStart = {0, 0, 0}, wp1 = {0, 0, 10}, wp2 = {10, 0, 10}, wp3 = {10, 10, 10}, wp4 = {0, 10, 10}, wp5 = {0, 0, 10}, wpFinal = {0, 0, 0};
    final float[][] wayPoints = {wpStart, wp1, wp2, wp3, wp4, wp5, wpFinal};
    // final float[] wp1 = {0, 0, 10}, wp2 = {0, 0, 0};
    // final float[][] wayPoints = {wp1, wp2};
    int currentWP = 0;

    QuadFrame quad;

    Controller(QuadFrame newQuad) {
        quad = newQuad;
    }

    void setAlt(float desiredAlt) {
        posZ.set(desiredAlt);
    }

    void update(float time, TableRow logRow) {
        angleX.set(posX.calculate(time, quad.posX));
        angleY.set(posY.calculate(time, quad.posY));
        float z = posZ.calculate(time, quad.posZ);
        float xAngle = angleX.calculate(time, quad.angleX);
        float yAngle = angleY.calculate(time, quad.angleY);
        float zAngle = angleZ.calculate(time, quad.angleZ);

        logRow.setFloat("setPosX", posX.setPoint);
        logRow.setFloat("setPosY", posY.setPoint);
        logRow.setFloat("setPosZ", posZ.setPoint);
        logRow.setFloat("setAngleX", angleX.setPoint);
        logRow.setFloat("setAngleY", angleY.setPoint);
        logRow.setFloat("setAngleZ", angleZ.setPoint);

        quad.motor1.setThrottle(z - xAngle + yAngle + zAngle);
        quad.motor2.setThrottle(z - xAngle - yAngle - zAngle);
        quad.motor3.setThrottle(z + xAngle - yAngle + zAngle);
        quad.motor4.setThrottle(z + xAngle + yAngle - zAngle);
    }

    boolean waypointNavigation() {
        if (quad.posX < posX.setPoint + accuracy && quad.posX > posX.setPoint - accuracy && quad.posY < posY.setPoint + accuracy && quad.posY > posY.setPoint - accuracy && quad.posZ < posZ.setPoint + accuracy && quad.posZ > posZ.setPoint - accuracy) {
            currentWP++;
            if (currentWP >= wayPoints.length) {
                println("Final Waypoint Reached.");
                return true;
            } else {
            println("Reached waypoint " + (currentWP - 1));
            posX.set(wayPoints[currentWP][0]);
            posY.set(wayPoints[currentWP][1]);
            posZ.set(wayPoints[currentWP][2]);
            return false;
            }
        } else if (quad.posZ <= 0 && currentWP != 0) {
            println("Landed");
            return true;
        }

        else {
            return false;
        }

    }


}
