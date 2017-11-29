QuadFrame quad = new QuadFrame(0, 0, 0, 0, 0, 0);
Jet jet = new Jet(0, 0, 0, 0, 0, 0);
Controller control = new Controller(quad);

// Timing
float timeStep = 0.005;
float runTime = 20;
float currentTime = 0;

// Loop control
boolean start = true;
int loopNumber = 0;
float maxLoops = runTime/timeStep;

// Plotting
int xPos;

void setup() {

    frameRate(1/timeStep);

    size(360, 240);
    background(0);

}

void draw() {

    if (start) {
        // Testing - check inertias
        // println("Quad Frame Inertias:\t" + quad.inertiaX + "\t" + quad.inertiaY + "\t" + quad.inertiaZ);
        // println("Jet Inertias:\t" + jet.inertiaX + "\t" + jet.inertiaY + "\t" + jet.inertiaZ);

        // Testing - check quad frame mass
        // println("Quad Frame Mass: " + quad.frameMass + " " + quad.rodMass + " " + (quad.motorMass*4) + " " + quad.mass);
        // exit();

        println("Time (s)\tHeight\t\tMotor1\tMotor2\tMotor3\tMotor4");

        control.setAlt(10);
        start = false;
        delay(1000);
    }

    quad.update(timeStep);
    control.update(timeStep);

    // println(quad.posX + "\t" + quad.posY + "\t" + quad.posZ + "\t" + quad.angleX + "\t" + quad.angleY + "\t" + quad.angleZ);
    println(nf(currentTime, 3, 2) + "\t" + nf(quad.posZ, 2, 2) + "\t\t" + quad.motor1.throttle + "\t" + quad.motor2.throttle + "\t" + quad.motor3.throttle + "\t" + quad.motor4.throttle);

    plot(quad.posZ);

    // Testing response to a disturbance
    if (currentTime > 10)
        quad.externalZForce = -1000;
    if (currentTime > 10.01)
        quad.externalZForce = 0;

    currentTime += timeStep;
    if (currentTime >= runTime) {
        exit();
    }
}

private void plot(float y) {

    stroke(127, 34, 255);
    line(xPos, height, xPos, height-(y*10));

    // at the edge of the screen, go back to the beginning:
    if (xPos >= width) {
      xPos = 0;
      background(0);
      stroke(255, 5, 5);
      line(0, height-control.setZ*10, width, height-control.setZ*10);
    } else {
      xPos++;
    }

}
