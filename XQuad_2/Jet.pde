class Jet extends PhysicalObject {

float jetHeight = 0.15, jetRadius = 0.041;
float mass = 4;
float thrust = mass*9.81;

GimbalServo gimbalX, gimbalY;
QuadFrame quad;

    Jet(float setPosX, float setPosY, float setPosZ, float setAngleX, float setAngleY, float setAngleZ, GimbalServo x, GimbalServo y) {
        posX = setPosX;
        posY = setPosY;
        posZ = setPosZ;
        angleX = setAngleX;
        angleY = setAngleY;
        angleZ = setAngleZ;
        connectServos(x, y);
        calculateParameters();

    }

    void connectServos(GimbalServo x, GimbalServo y) {
        gimbalX = x;
        gimbalY = y;
    }

    void connectQuad(QuadFrame newQuad) {
        quad = newQuad;
    }

    private void calculateParameters() {

        inertiaX = (mass/12)*(3*pow(jetRadius, 2) + pow(jetHeight, 2));
        inertiaY = inertiaX;
        inertiaZ = (mass*pow(jetRadius, 2));

        return;
    }

    void update(float timeStep, TableRow row) {
        calculateForces();
        calculateRotation(timeStep);

        row.setFloat("jetAngleX", angleX);
        row.setFloat("jetAngleY", angleY);
    }

    private void calculateForces() {
        forceX = thrust * sin(angleX) * cos(angleZ);
        forceY = thrust * sin(angleY) * cos(angleZ);
        forceZ = thrust * cos(angleX) * cos(angleY);

        // momentX = gimbalX.torque;
        // momentY = gimbalY.torque;
        // momentZ = 0;

    }

    private void calculateRotation(float time) {
        // angleAccelX = momentX / inertiaX;
        // angleAccelY = momentY / inertiaY;
        // angleAccelZ = momentZ / inertiaZ;
        //
        // angleVelocityX += (angleAccelX*time);
        // angleVelocityY += (angleAccelY*time);
        // angleVelocityZ += (angleAccelZ*time);

        angleX = gimbalX.position + quad.angleX;
        angleY = gimbalY.position + quad.angleY;
        angleZ = quad.angleZ;
    }

}
