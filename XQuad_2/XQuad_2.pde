QuadFrame quad = new QuadFrame();
Jet jet = new Jet();

void setup() {


}

void draw() {

    println("Quad Frame Inertias:\t" + quad.inertiaX + "\t" + quad.inertiaY + "\t" + quad.inertiaZ);
    println("Jet Inertias:\t" + jet.inertiaX + "\t" + jet.inertiaY + "\t" + jet.inertiaZ);
    exit();

}
