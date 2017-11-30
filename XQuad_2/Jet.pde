class Jet extends PhysicalObject {

float jetHeight = 0.15, jetRadius = 0.041;
float mass = 0.8;

    Jet(float setPosX, float setPosY, float setPosZ, float setAngleX, float setAngleY, float setAngleZ) {
        posX = setPosX;
        posY = setPosY;
        posZ = setPosZ;
        angleX = setAngleX;
        angleY = setAngleY;
        angleZ = setAngleZ;
        calculateParameters();

    }

    private void calculateParameters() {

        inertiaX = (mass/12)*(3*pow(jetRadius, 2) + pow(jetHeight, 2));
        inertiaY = inertiaX;
        inertiaZ = (mass*pow(jetRadius, 2));

        return;
    }


}
