// xQuad - Project Titan : Ismail Ahmad v4.0 Genetic
// Designed for engineers to test UAV's virtually before actually creating a prototype........ #ProjectTitan
// Currently uses standard PID Position ----> PID Angle controller.
// New Version - drag and lift now implemented.
// This version will try to implement a genetic algorithm to ooptimise a specific flight mission.

/*

  QuadX motor configuration

          4      2
           \    /
            \  /
             \/
             /\
            /  \
           /    \
          3      1

*/

import peasy.*;
PeasyCam cam;

/////////Genetic Algorithm/////////////
DNA[] population = new DNA[10];
Route route1;

/////////Initial Conditions////////////

float accelerationG = 9.81;
float torqueFactor = 1;
float density = 1.225;
float totalMass = 2.8;

/////////Quad Configuration Conditions////////////

float motor2Thrust = 0;
float motor3Thrust = 0;

float motor1Thrust = 0;
float motor4Thrust = 0;

float motorMaxThrust = 11;
float motorMinThrust = 0;

float armLength = 0.1;
float armWidth = 0.02;
float armMass =  0.03;
float armI;

float motorMass = 0.03;
float motorDiameter = 0.02;
float motorI;

float thetaZI, thetaYI, thetaXI = 0;
float thetaZO, thetaYO, thetaXO;
float omegaIZ, omegaIY, omegaIX;
float alphaZ, alphaY, alphaX;

float totalArmInertia;
float time = 0.01;
float yPosition;

float forceX, forceY, forceZ;
float accelerationX, accelerationY, accelerationZ;
float displacementXI, displacementYI, displacementZI;
float displacementXO, displacementYO, displacementZO;
float velocityXO, velocityYO, velocityZO = 0;
float velocityXI, velocityYI, velocityZI = 0;

/////////Angle in Degrees////////////////

float maxRoll = 20;
float maxPitch = 20;

/////////PID ANGLE Controller////////////

float setXA;
float setYA;
float setZA;

float kpXA = 0.0004;
float kpYA = 0.00009;
float kpZA = 0.0004;

float kiXA = 0.000001;
float kiYA = 0.000001;
float kiZA = 0.000001;

float kdXA = 0.005;
float kdYA = 0.001;
float kdZA = 0.005;

float errorXA, errorYA, errorZA;
float pXAError, pYAError, pZAError;
float integralXA, integralYA, integralZA;
float derivativeXA, derivativeYA, derivativeZA;
float lastErrorXA, lastErrorYA, lastErrorZA;

/////////PID POSITION Controller////////////

float setXP = 0;
float setYP = 200;
float setZP = 0;

float kpXP = 0.3;
float kpYP = 1.4;
float kpZP = 0.4;

float kiXP = 0.0001;
float kiYP = 0.001;
float kiZP = 0.0001;

float kdXP = 1;
float kdYP = 10;
float kdZP = 1.3;

float errorXP, errorYP, errorZP;
float pXError, pYError, pZError ;
float integralXP, integralYP, integralZP;
float derivativeXP, derivativeYP, derivativeZP;
float lastErrorXP, lastErrorYP, lastErrorZP;

/////////Drag Addition///////////////

float cDX, cDY, cDZ;
float surfaceA;
float dragX, dragY, dragZ;

/////////Lift Addition///////////////

float cLX, cLY, cLZ;
float liftX, liftY, liftZ;

/////////Arrays For Trail////////////

float[] trailPointsX = new float[200];
float[] trailPointsY = new float[200];
float[] trailPointsZ = new float[200];

int posX = 0;
int posY = 0;
int posZ = 0;

int i;
int trailCounter = 0;
int pos = 0;

float lapTime;
float timerNew, timerOld, timeInitial = 0;

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////



void setup(){

  cam = new PeasyCam(this,  500, 400, 0, 600);
  cam.setMinimumDistance(50);
  cam.setMaximumDistance(3000);

/////////Genetic Setup////////////////////////////////

  for (int i = 0; i < population.length; i++) {
    population[i] = new DNA();
  }

  route1 = new Route();

// float v = route1.xPoint[0];

/////////Max Roll and Pitch Angle/////////////////////

maxRoll = radians(maxRoll);
maxPitch = radians(maxPitch);

//////////////////////////////////////////////////////

  frameRate(1000);
  size(1000, 700, P3D);

/////////Drag Surface Area Calculations///////////////

  surfaceA = 4*(armWidth*armLength);

}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////


void draw(){

  millis();
  lights();
  background(250);

  pidAngleX();
  pidAngleY();
  pidAngleZ();

  pidPositionX();
  pidPositionY();
  pidPositionZ();

  calculateZAngle();
  calculateYAngle();
  calculateXAngle();

  dragCalculations();
  liftCalculations();

  motorCalculations();
  motorEndpoints();

  movementX();
  movementZ();
  movementY();


///////////////////////////////////////////////////////////


  yPosition = 600 + displacementYI;

  if ((yPosition) > 601)
  {
    yPosition = 600;
    displacementYI = 0;
    velocityYI = 0;
  }

///////////////////////////////////////////////////////////

  pushMatrix();
  translate(500 + displacementXI, yPosition, displacementZI);
  rotateZ(thetaZO);
  rotateX(thetaXO);
  rotateY(thetaYO + 0.785398);
  fill(100);
  box(armLength*100,1,armLength*10);

  trailCounter++;

  if (trailCounter == 99){

  trailPointsX[posX] = 500 + displacementXI;
  trailPointsY[posX] = yPosition;
  trailPointsZ[posX] = displacementZI;

  posX++;
  posY++;
  posZ++;

  trailCounter = 0;

  if (posX > 98)
  {
    posX = 0;
    posY = 0;
    posZ = 0;
  }
 }
  popMatrix();

//////////////////////////////

  pushMatrix();
  translate(500 + displacementXI, yPosition, displacementZI);
  rotateZ(thetaZO);
  rotateX(thetaXO);
  rotateY(thetaYO + 0.785398);
  fill(100);
  box(armLength*10,1,armLength*100);
  popMatrix();

  pushMatrix();
  translate(500 + setXP, 600 - setYP, -setZP);
  fill(230,0,0);
  box(5);
  popMatrix();

  pushMatrix();
  translate(500,600,0);
  fill(230);
  box(500,1,500);
  popMatrix();

  //println(displacementXI + "    " + displacementYI + "    " + displacementZI + "    " + frameRate);
  //println(velocityXI + "    " + velocityYI + "    " + velocityZI + "    " + frameRate);
  //println(liftX + "    " + liftZ + "    " + dragY + "    " + frameRate);

  for (i = 0; i < 200; i++)
  {
    pushMatrix();
    translate(trailPointsX[i], trailPointsY[i], trailPointsZ[i]);
    fill(255,20,147);
    box(2);
    popMatrix();
  }

  float distance = sqrt (((displacementXI-setXP)*(displacementXI-setXP)) + ((displacementZI+setZP)*(displacementZI+setZP)) + ((displacementYI+setYP)*(displacementYI+setYP)));

if (distance < 15) {

  timerNew = millis();
  println(timerNew);

     if (pos > 9){
     timerNew = millis() - timerOld;
     pos = 0;
     println("Lap Time = " + timerNew/1000);
     timerNew = timerOld;
     }

   setXP = route1.xPoint[pos];
   setZP = route1.zPoint[pos];
   setYP = route1.yPoint[pos];
   pos ++;
  }
}


///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

void calculateZAngle() {

  // : 2/3 since mass of both arms is included and multiplied by sin(0.785398) to account for length shortening in quadX configuration

  alphaZ = ((motor3Thrust+motor4Thrust) - (motor2Thrust+motor1Thrust))  / (((2/3)*armMass + 2*motorMass)*(armLength*sin(0.785398)));

  omegaIZ = omegaIZ + (alphaZ * time);

  thetaZO = thetaZI + omegaIZ*time + (1/2)*alphaZ*time*time;

  thetaZI = thetaZO;

}

void calculateXAngle() {

  alphaX = ((motor1Thrust+motor3Thrust) - (motor4Thrust+motor2Thrust) ) / (((2/3)*armMass + 2*motorMass)*armLength*sin(0.785398));

  omegaIX = omegaIX + (alphaX * time);

  thetaXO = thetaXI + omegaIX*time + (1/2)*alphaX*time*time;

  thetaXI = thetaXO;

}

void calculateYAngle() {

  alphaY = ((motor4Thrust+motor1Thrust)*torqueFactor - (motor3Thrust+motor2Thrust)*torqueFactor) / (((1/3)*(armMass*2+motorMass*2)*(2*armLength)*(2*armLength))+((armMass*2+motorMass*2)*armLength*armLength));

  omegaIY = omegaIY + (alphaY * time);

  thetaYO = thetaYI + omegaIY*time + (1/2)*alphaY*time*time;

  thetaYI = thetaYO;

}



///////////////////////////////////////////////////////////



void movementX() {

  forceX = ((motor1Thrust+motor2Thrust+motor3Thrust+motor4Thrust) * sin(thetaZO)) + dragX;

  if(forceX < -40){
    forceX = -40;
  }

  if (forceX > 40){
    forceX = 40;
  }

  accelerationX = forceX/totalMass;
  velocityXO = accelerationX*time;
  velocityXI = velocityXO+velocityXI;
  displacementXI = (velocityXI*time) + ((1/2)*accelerationX*time*time) + displacementXO;
  displacementXO = displacementXI;

}

//movementy checked
void movementY() {

  forceY = -((motor1Thrust+motor2Thrust+motor3Thrust+motor4Thrust) * sin((PI/2 - thetaZO))* sin((PI/2 - thetaXO))) + totalMass*accelerationG + dragY - liftX - liftY;

    if(forceY < -40){
    forceY = -40;
  }


  accelerationY = forceY/totalMass;
  velocityYO = accelerationY*time;
  velocityYI = velocityYO+velocityYI;
  displacementYI = velocityYI*time + (1/2)*accelerationY*time*time + displacementYO;
  displacementYO = displacementYI;

}


void movementZ() {

  forceZ =  -((motor1Thrust+motor2Thrust+motor3Thrust+motor4Thrust) * sin(thetaXO)) - dragZ  ;

  if(forceZ < -40)
  {
    forceZ = -40;
  }

  if (forceZ > 40)
  {
    forceZ = 40;
  }

  accelerationZ = forceZ/totalMass;
  velocityZO = accelerationZ*time;
  velocityZI = velocityZO+velocityZI;
  displacementZI = velocityZI*time + (1/2)*accelerationZ*time*time + displacementZO;
  displacementZO = displacementZI;

}


////////////////////////////////////////////////////////////////////////////////

////PID ANGLE CONTROLLER//////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

///done/////
void pidAngleX(){

 setXA = pZError;

 if (setXA < - maxPitch){
   setXA = - maxPitch;
 }

  if (setXA > maxPitch){
   setXA = maxPitch;
 }

 errorXA = setXA + thetaXO;

 integralXA = integralXA + errorXA*time;

 derivativeXA = (errorXA-lastErrorXA)/time;

 pXAError = kpXA*errorXA + kiXA*integralXA + kdXA*derivativeXA;

 lastErrorXA = errorXA;

}
//////done///////
void pidAngleZ(){

 setZA = -pXError;

 if (setZA < - maxRoll){
   setZA = - maxRoll;
 }

  if (setZA > maxRoll){
   setZA = maxRoll;
 }

 errorZA = setZA + thetaZO;

 integralZA = integralZA + errorZA*time;

 derivativeZA = (errorZA-lastErrorZA)/time;

 pZAError = kpZA*errorZA + kiZA*integralZA + kdZA*derivativeZA;

 lastErrorZA = errorZA;

}

void pidAngleY(){

 errorYA = setYA - thetaYO;

 integralYA = integralYA + errorYA*time;

 derivativeYA = (errorYA-lastErrorYA)/time;

 pYAError = kpYA*errorYA + kiYA*integralYA + kdYA*derivativeYA;

 lastErrorYA = errorYA;

}

/////////////////////////////////////////////////////////////////////////////////

////PID POSITION CONTROLLER//////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

void pidPositionY(){

 errorYP = setYP + displacementYI;

 integralYP = integralYP + (errorYP*time);

 derivativeYP = (errorYP-lastErrorYP)/time;

 pYError = (kpYP*errorYP + kiYP*integralYP + kdYP*derivativeYP);

 lastErrorYP = errorYP;

}

void pidPositionX(){

 errorXP = setXP - displacementXI;

 integralXP = integralXP + errorXP*time;

 derivativeXP = (errorXP-lastErrorXP)/time;

 pXError = (kpXP*errorXP + kiXP*integralXP + kdXP*derivativeXP);

 lastErrorXP = errorXP;

}

void pidPositionZ(){

 errorZP = -(setZP + displacementZI);

 integralZP = integralZP + errorZP*time;

 derivativeZP = (errorZP-lastErrorZP)/time;

 pZError = kpZP*errorZP + kiZP*integralZP + kdZP*derivativeZP;

 lastErrorZP = errorZP;

}

///////////////////////////////////////////////////////////

void motorCalculations(){

  motor1Thrust = pYError  - pXAError + pZAError + pYAError;

  motor2Thrust = pYError  + pXAError + pZAError - pYAError;

  motor3Thrust = pYError  - pXAError - pZAError - pYAError;

  motor4Thrust = pYError  + pXAError - pZAError + pYAError;

}
///////////////////////////////////////////////////////////

void motorEndpoints(){

      if (motor1Thrust < motorMinThrust )
  {
    motor1Thrust = motorMinThrust;
  }

      if (motor2Thrust < motorMinThrust)
    {
      motor2Thrust = motorMinThrust;
    }

        if (motor3Thrust < motorMinThrust)
      {
        motor3Thrust = motorMinThrust;
      }

          if (motor4Thrust < motorMinThrust)
        {
          motor4Thrust = motorMinThrust;
        }



    if (motor1Thrust > motorMaxThrust )
  {
    motor1Thrust = motorMaxThrust;
  }

      if (motor2Thrust > motorMaxThrust)
    {
      motor2Thrust = motorMaxThrust;
    }

        if (motor3Thrust > motorMaxThrust)
      {
        motor3Thrust = motorMaxThrust;
      }

          if (motor4Thrust > motorMaxThrust)
        {
          motor4Thrust = motorMaxThrust;
        }
}

void dragCalculations(){

  if (velocityXI > 0){
  cDX = 1.28 * sqrt(sin(thetaZO) * sin(thetaZO));
  dragX = -(0.5 * cDX * density * velocityXI * velocityXI * (surfaceA*sqrt(sin(thetaZO)*sin(thetaZO))));
  }
  else{
  cDX = 1.28 * sqrt(sin(thetaZO) * sin(thetaZO));
  dragX = (0.5 * cDX * density * velocityXI * velocityXI * (surfaceA*sqrt(sin(thetaZO)*sin(thetaZO))));
  }

  if (velocityZI > 0){
  cDZ = 1.28 * sqrt(sin(thetaXO) * sin(thetaXO));
  dragZ = -(0.5 * cDZ * density * velocityZI * velocityZI * (surfaceA*sqrt(sin(thetaXO)*sin(thetaXO))));
  }
  else{
  cDZ = 1.28 * sqrt(sin(thetaXO) * sin(thetaXO));
  dragZ = (0.5 * cDZ * density * velocityZI * velocityZI * (surfaceA*sqrt(sin(thetaXO)*sin(thetaXO))));
  }

  if (velocityYI > 0){
  cDY = ((1.28 * cos(thetaXO)) + (1.28 * cos(thetaZO)))/2 ;
  dragY = (0.5 * cDY * density * velocityYI * velocityYI * (surfaceA*sqrt(cos(thetaXO)*cos(thetaXO))*sqrt(cos(thetaZO)*cos(thetaZO))));
  }
  else{
  cDY = ((1.28 * cos(thetaXO)) + (1.28 * cos(thetaZO)))/2 ;
  dragY = -(0.5 * cDY * density * velocityYI * velocityYI * (surfaceA*sqrt(cos(thetaXO)*cos(thetaXO))*sqrt(cos(thetaZO)*cos(thetaZO))));
  }

}

void liftCalculations(){

  if (thetaZO > 0 && velocityXI > 0){
  cLX = 2 * PI * sqrt(thetaZO*thetaZO);
  liftX = (0.5 * cLX * density * velocityXI * velocityXI * (surfaceA*sqrt(cos(thetaZO)*cos(thetaZO))));
  }

  if (thetaZO < 0 && velocityXI > 0){
  cLX = 2* PI * sqrt(thetaZO*thetaZO);
  liftX = -(0.5 * cLX * density * velocityXI * velocityXI * (surfaceA*sqrt(cos(thetaZO)*cos(thetaZO))));
  }

  if (thetaZO > 0 && velocityXI < 0){
  cLX = 2* PI * sqrt(thetaZO*thetaZO);
  liftX = -(0.5 * cLX * density * velocityXI * velocityXI * (surfaceA*sqrt(cos(thetaZO)*cos(thetaZO))));
  }

  if (thetaZO < 0 && velocityXI < 0){
  cLX = 2* PI * sqrt(thetaZO*thetaZO);
  liftX = (0.5 * cLX * density * velocityXI * velocityXI * (surfaceA*sqrt(cos(thetaZO)*cos(thetaZO))));
  }



  if (thetaXO > 0 && velocityZI > 0){
  cLX = 2 * PI * sqrt(thetaXO*thetaXO);
  liftZ = (0.5 * cLX * density * velocityZI * velocityZI * (surfaceA*sqrt(cos(thetaXO)*cos(thetaXO))));
  }

  if (thetaXO < 0 && velocityZI > 0){
  cLX = 2* PI * sqrt(thetaXO*thetaXO);
  liftZ = -(0.5 * cLX * density * velocityZI * velocityZI * (surfaceA*sqrt(cos(thetaXO)*cos(thetaXO))));
  }

  if (thetaXO > 0 && velocityZI < 0){
  cLX = 2* PI * sqrt(thetaXO*thetaXO);
  liftZ = -(0.5 * cLX * density * velocityZI * velocityZI * (surfaceA*sqrt(cos(thetaXO)*cos(thetaXO))));
  }

  if (thetaXO < 0 && velocityZI < 0){
  cLX = 2* PI * sqrt(thetaXO*thetaXO);
  liftZ = (0.5 * cLX * density * velocityZI * velocityZI * (surfaceA*sqrt(cos(thetaXO)*cos(thetaXO))));
  }

}
