import peasy.*;

PeasyCam cam;

// Run in which mode?
// Accepted inputs: HEIGHT, XANGLE, YANGLE, XPOS, JETX
final String plottedVariable = "XPOS";
final float startingHeight = 0;

// Saving data
// WARNING will overwrite old data if given same name
Table log = new Table();
String fileName = "Jet gimbal square path 4";
final boolean save = false;

// Timing
final float timeStep = 0.005;
final float runTime = 60;
final float eventTime = 5; // when the disturbance is applied
final float eventLength = 0.01;
float currentTime = 0;

// Plotting
int xPos;
final int plotSensitivity = 10; // pixel height = value*plotSensitivity
                                // use ~10 for pos, ~500 for angle

// Objects
Quad quad = new Quad(0, 0, startingHeight, 0, 0, 0);

Controller control = new Controller(quad); // TODO update with new objects

// Loop control
int loopNumber = 0;
final float maxLoops = runTime/timeStep;
boolean finish = false;

void setup() {

    frameRate(1/timeStep);

    size(720, 360);
    background(0);

    log.addColumn("time");
    log.addColumn("posX");
    log.addColumn("setPosX");
    log.addColumn("posY");
    log.addColumn("setPosY");
    log.addColumn("posZ");
    log.addColumn("setPosZ");
    log.addColumn("angleX");
    log.addColumn("setAngleX");
    log.addColumn("angleY");
    log.addColumn("setAngleY");
    log.addColumn("angleZ");
    log.addColumn("setAngleZ");
    log.addColumn("m1");
    log.addColumn("m2");
    log.addColumn("m3");
    log.addColumn("m4");
    log.addColumn("jetAngleX");
    log.addColumn("jetAngleY");
    log.addColumn("gimbalXTorque");
    log.addColumn("gimbalYTorque");


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
    control.update(timeStep, logRow);
    finish = control.waypointNavigation();

/////Display Data /////
    println(nf(currentTime, 3, 2) + "\t" + quad.posX + "\t" + quad.posY + "\t" + quad.posZ + "\t" + quad.quad.angleX + "\t" + quad.quad.angleY + "\t" + quad.quad.angleZ);
    // println(nf(currentTime, 3, 2) + "\t" + nf(quad.posZ, 2, 2) + "\t\t" + quad.motor1.throttle + "\t" + quad.motor2.throttle + "\t" + quad.motor3.throttle + "\t" + quad.motor4.throttle);
    // println(control.posX.setPoint + "\t" + control.posY.setPoint + "\t" + control.posZ.setPoint + "\t" + quad.posX + "\t" + quad.posY + "\t" + quad.posZ);

    switch (plottedVariable) {
        case "HEIGHT":
        plot(quad.posZ, control.posZ.setPoint);
        break;

        case "XANGLE":
        plot(quad.quad.angleX, control.angleX.setPoint);
        break;

        case "YANGLE":
        plot(quad.quad.angleY, control.angleY.setPoint);
        break;

        case "XPOS":
        plot(quad.posX, control.posX.setPoint);
        break;

        case "JETX":
        plot(quad.jet.angleX, quad.gimbalX.angle.setPoint);
        break;

        default:
        println("PLOTTED VARIABLE SELECTION ERROR");
        exit();
        break;
    }

///// Testing response to a disturbance /////
    if (currentTime > eventTime) {
        // quad.externalZForce = -1000;
        // quad.externalXMoment = 100;
        // quad.externalYMoment = 100;
    }
    if (currentTime > (eventTime + eventLength)) {
        quad.noForces();
    }

///// End /////
    currentTime += timeStep;
    if (currentTime >= runTime) {
        finish = true;
    }
    if (finish) {
        if (save)
            saveTable(log, "data/" + fileName + ".csv");
        exit();
    }

}

private void plot(float y, float setPoint) {

    stroke(127, 34, 255);
    line(xPos, height/2, xPos, height/2-(y*plotSensitivity));
    stroke(255, 5, 5);
    line(0, (height/2)-setPoint*plotSensitivity, width, (height/2)-setPoint*plotSensitivity);

    // at the edge of the screen, go back to the beginning:
    if (xPos >= width) {
      xPos = 0;
      background(0);
    } else {
      xPos++;
    }

}
