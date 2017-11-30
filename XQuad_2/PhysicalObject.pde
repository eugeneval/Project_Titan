class PhysicalObject {

// Cartesian parameters
float mass;
float posX, posY, posZ;
float velocityX, velocityY, velocityZ;
float accelX, accelY, accelZ;
float forceX, forceY, forceZ;
float externalXForce, externalYForce, externalZForce;

// Angular parameters
float inertiaX, inertiaY, inertiaZ;
float angleX, angleY, angleZ;
float angleVelocityX, angleVelocityY, angleVelocityZ;
float angleAccelX, angleAccelY, angleAccelZ;
float momentX, momentY, momentZ;
float externalXMoment, externalYMoment, externalZMoment;

    PhysicalObject() {

        calculateParameters();

    }

    private void calculateParameters() {

        return;
    }

    void noForces() {
        externalXForce = 0;
        externalYForce = 0;
        externalZForce = 0;
        externalXMoment = 0;
        externalYMoment = 0;
        externalZMoment = 0;
    }

}
