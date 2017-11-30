class Motor {

private float force = 0;
private float minForce, maxForce;

float throttle = 0;

    Motor(float min, float max) {
        minForce = min;
        maxForce = max;
    }

    // set throttle
    void setThrottle(float setThrottle) {
        if (setThrottle > 1) {
            setThrottle = 1;
        }
        if (setThrottle < 0) {
            setThrottle = 0;
        }

        throttle = setThrottle;
        force = (maxForce - minForce)*(throttle);
    }

    float force() {
        return force;
    }


}
