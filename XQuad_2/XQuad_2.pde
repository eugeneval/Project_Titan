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

//Saving data
Table log = new Table();
String fileName = "movement";

void setup() {

    frameRate(1/timeStep);

    size(720, 240);
    background(0);

    log.addColumn("time");
    log.addColumn("posX");
    log.addColumn("posY");
    log.addColumn("posZ");
    log.addColumn("angleX");
    log.addColumn("angleY");
    log.addColumn("angleZ");
    log.addColumn("m1");
    log.addColumn("m2");
    log.addColumn("m3");
    log.addColumn("m4");
}

void draw() {

    TableRow logRow = log.addRow();
    logRow.setFloat("time", currentTime);

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

    quad.update(timeStep, logRow);
    control.update(timeStep);

    // println(quad.posX + "\t" + quad.posY + "\t" + quad.posZ + "\t" + quad.angleX + "\t" + quad.angleY + "\t" + quad.angleZ);
    println(nf(currentTime, 3, 2) + "\t" + nf(quad.posZ, 2, 2) + "\t\t" + quad.motor1.throttle + "\t" + quad.motor2.throttle + "\t" + quad.motor3.throttle + "\t" + quad.motor4.throttle);

    plot(quad.posZ, control.posZ.setPoint);

    // Testing response to a disturbance
    if (currentTime > 10)
        quad.externalZForce = -1000;
    if (currentTime > 10.01)
        quad.externalZForce = 0;

    currentTime += timeStep;
    if (currentTime >= runTime) {
        saveTable(log, "data/" + fileName + ".csv");
        exit();
    }
}

private void plot(float y, float setPoint) {

    stroke(127, 34, 255);
    line(xPos, height, xPos, height-(y*10));
    stroke(255, 5, 5);
    line(0, height-setPoint*10, width, height-setPoint*10);

    // at the edge of the screen, go back to the beginning:
    if (xPos >= width) {
      xPos = 0;
      background(0);
    } else {
      xPos++;
    }

}
