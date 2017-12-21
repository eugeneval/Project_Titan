class Quad extends PhysicalObject {

    GimbalServo gimbalX, gimbalY;
    QuadFrame quad;
    Jet jet;

    Quad(float setPosX, float setPosY, float setPosZ, float setAngleX, float setAngleY, float setAngleZ) {
        posX = setPosX;
        posY = setPosY;
        posZ = setPosZ;
        angleX = setAngleX;
        angleY = setAngleY;
        angleZ = setAngleZ;

        gimbalX = new GimbalServo(0.8, 0.01, 1, radians(45));
        gimbalY = new GimbalServo(0.8, 0.01, 1, radians(45));
        quad = new QuadFrame(posX, posY, posZ, angleX, angleY, angleZ, gimbalX, gimbalY);
        jet = new Jet(posX, posY, posZ, angleX, angleY, angleZ, gimbalX, gimbalY);

        jet.connectQuad(quad);
        quad.connectJet(jet);

        gimbalX.addJetInertia(jet.inertiaX, jet.mass, 0.01);
        gimbalY.addJetInertia(jet.inertiaX, jet.mass, 0.01);

        calculateParameters();
    }

    void calculateParameters() {

        mass = quad.mass + jet.mass;

    }

    void update(float time, TableRow log) {
        gimbalX.update(time, jet.angleX);
        gimbalY.update(time, jet.angleY);
        quad.update(time, log);
        jet.update(time, log);

        calculateForces();
        calculateMovement(timeStep);

        log.setFloat("posX", posX);
        log.setFloat("posY", posY);
        log.setFloat("posZ", posZ);
        log.setFloat("gimbalXTorque", gimbalX.torque);
        log.setFloat("gimbalYTorque", gimbalY.torque);
    }

    private void calculateForces() {

        forceX = quad.forceX + jet.forceX;
        forceY = quad.forceY + jet.forceY;
        forceZ = quad.forceZ + jet.forceZ;

        // Gravity
        forceZ -= (9.81 * mass);

        forceX += externalXForce;
        forceY += externalYForce;
        forceZ += externalZForce;
    }

    private void calculateMovement(float time) {
        accelX = forceX / mass;
        accelY = forceY / mass;
        accelZ = forceZ / mass;

        velocityX += (accelX*time);
        velocityY += (accelY*time);
        velocityZ += (accelZ*time);

        posZ += (velocityZ*time);
        if (posZ < 0) {
            posZ = 0;
        }

        posX += (velocityX*time);
        posY += (velocityY*time);
    }

}
