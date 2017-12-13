class GimbalServo {

    PID_Values angle;
    float position, torque;
    private float accel, velocity, inertia;
    private final float servoInertia = 3.89E-4, servoMaxVelocity = 8.73;
    final float mass = 0.079;

    GimbalServo(float kP, float kI, float kD, float maxAngle) {
        angle = new PID_Values(kP, kI, kD, maxAngle);
        angle.set(0);
    }

    void addJetInertia(float jetInertia, float jetMass, float armLength) {
        inertia = servoInertia;
        inertia += jetInertia + (jetMass * pow(armLength, 2));
    }

    void update(float time, float error) {
        torque = angle.calculate(time, error);
        // TODO: max torque
        calculateMovement(time, torque);
    }

    private void calculateMovement(float time, float torque) {
        accel = torque/inertia;
        velocity += (accel*time);
        if (velocity > servoMaxVelocity) {
            velocity = servoMaxVelocity;
        } else if (velocity < -servoMaxVelocity) {
            velocity = -servoMaxVelocity;
        }

        position += (velocity*time);

    }

}
