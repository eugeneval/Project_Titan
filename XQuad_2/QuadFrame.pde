class QuadFrame extends PhysicalObject {

///////////////////////////////////////////////////////////////////////////////
// Parameters
//////////////////////////////////////////////////////////////////////////////

final private float frameInnerRadius = 0.18, frameOuterRadius = 0.2, frameHeight = 0.005, rodLength = 0.1, rodRadius = 0.002;
final private float density = 2700;
float frameMass, rodMass, motorMass = 0.05;

final private float motorMinForce = 0, motorMaxForce = 10, motorTorqueConstant = 1;
Motor motor1 = new Motor(motorMinForce, motorMaxForce);
Motor motor2 = new Motor(motorMinForce, motorMaxForce);
Motor motor3 = new Motor(motorMinForce, motorMaxForce);
Motor motor4 = new Motor(motorMinForce, motorMaxForce);

float externalXForce, externalYForce, externalZForce;

///////////////////////////////////////////////////////////////////////////////
// Constructor
//////////////////////////////////////////////////////////////////////////////
    QuadFrame(float setPosX, float setPosY, float setPosZ, float setAngleX, float setAngleY, float setAngleZ) {
        posX = setPosX;
        posY = setPosY;
        posZ = setPosZ;
        angleX = setAngleX;
        angleY = setAngleY;
        angleZ = setAngleZ;
        calculateParameters();

    }

    private void calculateParameters() {

        frameMass = density * frameHeight * PI * (frameOuterRadius - frameInnerRadius);
        rodMass = density * rodLength * PI * rodRadius;
        mass = frameMass + rodMass + motorMass;

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

///////////////////////////////////////////////////////////////////////////////
// UPDATE
//////////////////////////////////////////////////////////////////////////////
    void update(float timeStep, TableRow row) {
        calculateForces();
        calculateMovement(timeStep);

        row.setFloat("posX", posX);
        row.setFloat("posY", posY);
        row.setFloat("posZ", posZ);
        row.setFloat("angleX", angleX);
        row.setFloat("angleY", angleY);
        row.setFloat("angleZ", angleZ);
        row.setFloat("m1", motor1.throttle);
        row.setFloat("m2", motor2.throttle);
        row.setFloat("m3", motor3.throttle);
        row.setFloat("m4", motor4.throttle);
    }


    private void calculateForces() {

        forceX = (motor1.force() + motor2.force() + motor3.force() + motor4.force()) * sin(angleX) * cos(angleZ);
        forceY = (motor1.force() + motor2.force() + motor3.force() + motor4.force()) * sin(angleY) * cos(angleZ);
        forceZ = (motor1.force() + motor2.force() + motor3.force() + motor4.force()) * cos(angleX) * cos(angleY);
        forceZ -= (9.81 * mass); // Don't forget about gravity!

        forceX += externalXForce;
        forceY += externalYForce;
        forceZ += externalZForce;

        momentX = (-motor1.force() - motor2.force() + motor3.force() + motor4.force()) * (rodLength + frameOuterRadius) * cos(radians(45));
        momentY = (motor1.force() - motor2.force() - motor3.force() + motor4.force()) * (rodLength + frameOuterRadius) * cos(radians(45));
        momentZ = (motor1.force() - motor2.force() + motor3.force() - motor4.force()) * (rodLength + frameOuterRadius) * cos(radians(45)) * motorTorqueConstant;

    }

    private void calculateMovement(float time) {

        accelX = forceX / mass;
        accelY = forceY / mass;
        accelZ = forceZ / mass;
        angleAccelX = momentX / inertiaX;
        angleAccelY = momentY / inertiaY;
        angleAccelZ = momentZ / inertiaZ;

        velocityX += (accelX*time);
        velocityY += (accelY*time);
        velocityZ += (accelZ*time);
        angleVelocityX += (angleAccelX*time);
        angleVelocityY += (angleAccelY*time);
        angleVelocityZ += (angleAccelZ*time);

        posX += (velocityX*time);
        posY += (velocityY*time);
        posZ += (velocityZ*time);
        angleX += (angleVelocityX*time);
        angleY += (angleVelocityY*time);
        angleZ += (angleVelocityZ*time);

        // Cannot go below ground level
        if (posZ < 0) {
            posZ = 0;
        }
    }

}
