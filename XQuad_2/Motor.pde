class Motor {

private float force = 0;
private float minForce, maxForce;

    Motor(float min, float max) {
        minForce = min;
        maxForce = max;
    }

    // set throttle as a percentage
    void setThrottle(float throttle) {
        force = (maxForce - minForce)*(throttle/100);
    }

    float force() {
        return force;
    }


}
