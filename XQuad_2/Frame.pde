class QuadFrame {

///////////////////////////////////////////////////////////////////////////////
// Parameters
//////////////////////////////////////////////////////////////////////////////

private float frameInnerRadius = 0.18, frameOuterRadius = 0.2, frameHeight = 0.02, rodLength = 0.2, rodRadius = 0.005;
private float frameMass, rodMass, motorMass = 0.05;
private float density = 2700;
float inertiaX, inertiaY, inertiaZ;

    QuadFrame() {

        calculateParameters();

    }

    private void calculateParameters() {

        frameMass = density * frameHeight * PI * (frameOuterRadius - frameInnerRadius);
        rodMass = density * rodLength * PI * rodRadius;

        float frameInertiaX = ((PI*density*frameHeight)/12) * ((3*(pow(frameOuterRadius, 4) - pow(frameInnerRadius, 4))) + pow(frameHeight, 2)*(pow(frameOuterRadius, 2) - pow(frameInnerRadius, 2)));
        float frameInertiaY = frameInertiaX;
        float frameInertiaZ = ((PI*density*frameHeight)/2) * (pow(frameOuterRadius, 4) - pow(frameInnerRadius, 4));

        float rodInertiaX = sin(radians(45)) * (((rodMass * pow(rodLength, 2))/12) + (rodMass)*pow((frameOuterRadius + rodLength/2), 2));
        float rodInertiaY = rodInertiaX;
        float rodInertiaZ = ((rodMass * pow(rodLength, 2))/12) + (rodMass)*pow((frameOuterRadius + rodLength/2), 2);

        float motorInertiaX = sin(radians(45)) * motorMass * pow((frameOuterRadius + rodLength), 2);
        float motorInertiaY = motorInertiaX;
        float motorInertiaZ = motorMass * pow((frameOuterRadius + rodLength), 2);

        inertiaX = frameInertiaX + 4*rodInertiaX + 4*motorInertiaX;
        inertiaY = frameInertiaY + 4*rodInertiaY + 4*motorInertiaY;
        inertiaZ = frameInertiaZ + 4*rodInertiaZ + 4*motorInertiaZ;

        return;

    }


}
