class PhysicalObject {

// Cartesian parameters
float mass;
float posX, posY, posZ;
float velocityX, velocityY, velocityZ;
float accelX, accelY, accelZ;
float forceX, forceY, forceZ;

// Angular parameters
float inertiaX, inertiaY, inertiaZ;
float angleX, angleY, angleZ;
float angleVelocityX, angleVelocityY, angleVelocityZ;
float angleAccelX, angleAccelY, angleAccelZ;
float momentX, momentY, momentZ;

    PhysicalObject() {

        calculateParameters();

    }

    private void calculateParameters() {

        return;
    }

}
