QuadFrame quad = new QuadFrame(0, 0, 0, 0, 0, 0);
Jet jet = new Jet(0, 0, 0, 0, 0, 0);

float timeStep = 0.01;

boolean start = true;
int loopNumber = 0;
int maxLoops = 100;

void setup() {

    frameRate = (1/timeStep);

}

void draw() {

    if (start) {
        println("Quad Frame Inertias:\t" + quad.inertiaX + "\t" + quad.inertiaY + "\t" + quad.inertiaZ);
        println("Jet Inertias:\t" + jet.inertiaX + "\t" + jet.inertiaY + "\t" + jet.inertiaZ);
        delay(1000);
        start = false;
    }
    quad.update(timeStep);
    println(quad.posX + "\t" + quad.posY + "\t" + quad.posZ + "\t" + quad.angleX + "\t" + quad.angleY + "\t" + quad.angleZ);
    loopNumber++;
    if (loopNumber > maxLoops) {
        exit();
    }


}
