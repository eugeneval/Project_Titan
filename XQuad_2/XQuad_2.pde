import peasy.*;

PeasyCam cam;

// Run in which mode?
// Accepted inputs: PLOT, 3D
final String simRunMode = "PLOT";
// Accepted inputs: HEIGHT, XANGLE
final String plottedVariable = "HEIGHT";
final float startingHeight = 0;

// Saving data
Table log = new Table();
String fileName = "X and Y angle disturbance";
final boolean save = false;

// Timing
final float timeStep = 0.005;
final float runTime = 20;
float currentTime = 0;

// Objects
QuadFrame quad = new QuadFrame(0, 0, startingHeight, 0, 0, 0);
Jet jet = new Jet(0, 0, 0, 0, 0, 0);
Controller control = new Controller(quad);

// Loop control
int loopNumber = 0;
final float maxLoops = runTime/timeStep;

// Plotting
int xPos;

void setup() {

    frameRate(1/timeStep);

    // if (simRunMode == "3D") {
    //     size(1000, 700, P3D);
    //     background(250);
    //     rectMode(CENTER);
    //
    //     cam = new PeasyCam(this, -400, -400, 0, 600);
    //     cam.setMinimumDistance(50);
    //     cam.setMaximumDistance(500);
    // } else if (simRunMode == "PLOT") {
    //     size(720, 360);
    //     background(250);
    // }

    size(720, 360);
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

    // Testing - check inertias
    // println("Quad Frame Inertias:\t" + quad.inertiaX + "\t" + quad.inertiaY + "\t" + quad.inertiaZ);
    // println("Jet Inertias:\t" + jet.inertiaX + "\t" + jet.inertiaY + "\t" + jet.inertiaZ);

    // Testing - check quad frame mass
    // println("Quad Frame Mass: " + quad.frameMass + " " + quad.rodMass + " " + (quad.motorMass*4) + " " + quad.mass);
    // exit();

    println("Time (s)\tHeight\t\tMotor1\tMotor2\tMotor3\tMotor4");

    control.setAlt(10);
    delay(1000);
}

void draw() {

///// Update and log /////
    TableRow logRow = log.addRow();
    logRow.setString("time", nf(currentTime, 3, 3));
    quad.update(timeStep, logRow);
    control.update(timeStep);

/////Display Data /////
    // println(quad.posX + "\t" + quad.posY + "\t" + quad.posZ + "\t" + quad.angleX + "\t" + quad.angleY + "\t" + quad.angleZ);
    println(nf(currentTime, 3, 2) + "\t" + nf(quad.posZ, 2, 2) + "\t\t" + quad.motor1.throttle + "\t" + quad.motor2.throttle + "\t" + quad.motor3.throttle + "\t" + quad.motor4.throttle);

    if (simRunMode == "PLOT") {
        switch (plottedVariable) {
            case "HEIGHT":
            plot(quad.posZ, control.posZ.setPoint);
            break;

            case "XANGLE":
            plot(quad.angleX, control.angleX.setPoint);
            break;

            default:
            println("PLOTTED VARIABLE SELECTION ERROR");
            exit();
            break;
        }
    }

//// 3D representation /////
    if (simRunMode == "3D") {
        lights();
        background(255);

        pushMatrix();
        fill(250);
        rotateX(radians(90));
        rect(0, 0, 1000, 1000);
        popMatrix();

        pushMatrix();
        translate(quad.posY, -quad.posZ, -quad.posX);
        rotateZ(quad.angleY);
        rotateX(quad.angleX);
        rotateY(quad.angleZ);
        fill(100);
        box(40);
        popMatrix();
    }

///// Testing response to a disturbance /////
    if (currentTime > 10) {
        // quad.externalZForce = -1000;
        // quad.externalXMoment = 100;
        // quad.externalYMoment = 100;
    }
    if (currentTime > 10.01) {
        quad.noForces();
    }

///// End /////
    currentTime += timeStep;
    if (currentTime >= runTime) {
        if (save)
            saveTable(log, "data/" + fileName + ".csv");
        exit();
    }
}

private void plot(float y, float setPoint) {

    stroke(127, 34, 255);
    line(xPos, height/2, xPos, height/2-(y*10));
    stroke(255, 5, 5);
    line(0, (height/2)-setPoint*10, width, (height/2)-setPoint*10);

    // at the edge of the screen, go back to the beginning:
    if (xPos >= width) {
      xPos = 0;
      background(0);
    } else {
      xPos++;
    }

}
