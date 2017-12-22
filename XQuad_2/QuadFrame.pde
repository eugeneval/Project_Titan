class QuadFrame extends PhysicalObject {

///////////////////////////////////////////////////////////////////////////////
// Parameters
//////////////////////////////////////////////////////////////////////////////

final private float frameInnerRadius = 0.18, frameOuterRadius = 0.2, frameHeight = 0.005, rodLength = 0.1, rodRadius = 0.01;
final private float density = 2700;
float frameMass, rodMass, motorMass = 0.05;

final private float motorMinForce = 0, motorMaxForce = 15, motorTorqueConstant = 1;
Motor motor1 = new Motor(motorMinForce, motorMaxForce);
Motor motor2 = new Motor(motorMinForce, motorMaxForce);
Motor motor3 = new Motor(motorMinForce, motorMaxForce);
Motor motor4 = new Motor(motorMinForce, motorMaxForce);

GimbalServo gimbalX, gimbalY;
Jet jet;

///////////////////////////////////////////////////////////////////////////////
// Constructor
//////////////////////////////////////////////////////////////////////////////
    QuadFrame(float setPosX, float setPosY, float setPosZ, float setAngleX, float setAngleY, float setAngleZ, GimbalServo x, GimbalServo y) {
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

    void connectJet(Jet newJet) {
        jet = newJet;
    }

    private void calculateParameters() {

        frameMass = density * frameHeight * PI * (frameOuterRadius - frameInnerRadius);
        rodMass = density * rodLength * PI * pow(rodRadius, 2);
        mass = frameMass + 4*rodMass + 4*motorMass + 2*gimbalX.mass; 

        float frameInertiaX = ((PI*density*frameHeight)/12) * ((3*(pow(frameOuterRadius, 4) - pow(frameInnerRadius, 4))) + pow(frameHeight, 2)*(pow(frameOuterRadius, 2) - pow(frameInnerRadius, 2)));
        float frameInertiaY = frameInertiaX;
        float frameInertiaZ = ((PI*density*frameHeight)/2) * (pow(frameOuterRadius, 4) - pow(frameInnerRadius, 4));

        float rodInertiaX = sin(radians(45)) * (((rodMass * pow(rodLength, 2))/12) + (rodMass)*pow((frameOuterRadius + rodLength/2), 2));
        float rodInertiaY = rodInertiaX;
        float rodInertiaZ = ((rodMass * pow(rodLength, 2))/12) + (rodMass)*pow((frameOuterRadius + rodLength/2), 2);

        float motorInertiaX = sin(radians(45)) * motorMass * pow((frameOuterRadius + rodLength), 2);
        float motorInertiaY = motorInertiaX;
        float motorInertiaZ = motorMass * pow((frameOuterRadius + rodLength), 2);

        float servoInertiaX = (gimbalX.mass * frameInnerRadius * sin(45));
        float servoInertiaY = (gimbalX.mass * frameInnerRadius * sin(45));
        float servoInertiaZ = (gimbalX.mass * frameInnerRadius);

        inertiaX = frameInertiaX + 4*rodInertiaX + 4*motorInertiaX + 2*servoInertiaX;
        inertiaY = frameInertiaY + 4*rodInertiaY + 4*motorInertiaY + 2*servoInertiaY;
        inertiaZ = frameInertiaZ + 4*rodInertiaZ + 4*motorInertiaZ + 2* servoInertiaZ;

        return;

    }

///////////////////////////////////////////////////////////////////////////////
// UPDATE
//////////////////////////////////////////////////////////////////////////////
    void update(float timeStep, TableRow row) {
        calculateForces();
        calculateRotation(timeStep);

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

        momentX = (-motor1.force() - motor2.force() + motor3.force() + motor4.force()) * (rodLength + frameOuterRadius) * cos(radians(45));
        momentY = (motor1.force() - motor2.force() - motor3.force() + motor4.force()) * (rodLength + frameOuterRadius) * cos(radians(45));
        momentZ = (motor1.force() - motor2.force() + motor3.force() - motor4.force()) * (rodLength + frameOuterRadius) * cos(radians(45)) * motorTorqueConstant;

        momentZ += gimbalX.torque * frameInnerRadius;
        momentZ += gimbalY.torque * frameInnerRadius;

        momentX += externalXMoment;
        momentY += externalYMoment;
        momentZ += externalZMoment;

    }

    private void calculateRotation(float time) {

        angleAccelX = momentX / inertiaX;
        angleAccelY = momentY / inertiaY;
        angleAccelZ = momentZ / inertiaZ;

        angleVelocityX += (angleAccelX*time);
        angleVelocityY += (angleAccelY*time);
        angleVelocityZ += (angleAccelZ*time);

        // Cannot go below ground level, if at ground level cannot move other than up
        // if (posZ < 0) {
        //     posZ = 0;
        //     velocityX = 0;
        //     velocityY = 0;
        //     angleVelocityX = 0;
        //     angleVelocityY = 0;
        //     angleVelocityZ = 0;
        // } // TODO: is this needed?

        angleX += (angleVelocityX*time);
        angleY += (angleVelocityY*time);
        angleZ += (angleVelocityZ*time);
    }

}
