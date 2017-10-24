import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import peasy.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class XQuad extends PApplet {

// xQuad - Project Titan : Ismail Ahmad v4.2 Genetic
// Designed for engineers to test UAV's virtually before actually creating a prototype........ #ProjectTitan
// Currently uses standard PID Position ----> PID Angle controller.
// New Version - drag and lift currently commented out.
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


PeasyCam cam;

/////////Genetic Algorithm/////////////

DNA[] population = new DNA[10];
int DNAPos = 0;
float[] fitArray = new float[10];
float fitArrayMax = 0;
float fitArrayMin = 0;
float[] fitPopulation = new float[10];
float totalFitPopulation = 0;
float[] newPopulation = new float[10];
float totalNewPopulation = 0;
DNA[] newPopulationFinal = new DNA[120];
int selection1, selection2;
int count = 0;
int generationNo;

/////////Initial Conditions////////////

float accelerationG = 9.81f;
float torqueFactor = 1;
float density = 1.225f;
float totalMass = 2.8f;

/////////Quad Configuration Conditions////////////

float motor2Thrust = 0;
float motor3Thrust = 0;

float motor1Thrust = 0;
float motor4Thrust = 0;

float motorMaxThrust = 11;
float motorMinThrust = 0;

float armLength = 0.1f;
float armWidth = 0.02f;
float armMass =  0.03f;
float armI;

float motorMass = 0.03f;
float motorDiameter = 0.02f;
float motorI;

float thetaZI, thetaYI, thetaXI = 0;
float thetaZO, thetaYO, thetaXO;
float omegaIZ, omegaIY, omegaIX;
float alphaZ, alphaY, alphaX;

float totalArmInertia;
float time = 0.01f;
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

float kpXA = 0.0004f;
float kpYA = 0.00009f;
float kpZA = 0.0004f;

float kiXA = 0.000001f;
float kiYA = 0.000001f;
float kiZA = 0.000001f;

float kdXA = 0.005f;
float kdYA = 0.001f;
float kdZA = 0.005f;

float errorXA, errorYA, errorZA;
float pXAError, pYAError, pZAError;
float integralXA, integralYA, integralZA;
float derivativeXA, derivativeYA, derivativeZA;
float lastErrorXA, lastErrorYA, lastErrorZA;

/////////PID POSITION Controller////////////

float setXP = 10;
float setYP = 10;
float setZP = 10;

float kpXP = 0.3f;
float kpYP = 1.4f;
float kpZP = 0.4f;

float kiXP = 0.0001f;
float kiYP = 0.001f;
float kiZP = 0.0001f;

float kdXP = 1;
float kdYP = 10;
float kdZP = 1.3f;

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
float timerNew1, timerOld1;


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
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////


public void setup(){

  println("asfcadsvwef");
  cam = new PeasyCam(this,  500, 400, 0, 600);
  cam.setMinimumDistance(50);
  cam.setMaximumDistance(3000);

/////////Genetic Setup////////////////////////////////

  for (int i = 0; i < population.length; i++) {
    population[i] = new DNA();
  }

///////////////////////////////////////////////////////////

/////////Max Roll and Pitch Angle/////////////////////

maxRoll = radians(maxRoll);
maxPitch = radians(maxPitch);

//////////////////////////////////////////////////////

  frameRate(1000);
  

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
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////


public void draw(){

  kpXA = population[DNAPos].genes[0];
  kpYA = population[DNAPos].genes[1];
  kpZA = population[DNAPos].genes[2];

  kiXA = population[DNAPos].genes[3];
  kiYA = population[DNAPos].genes[4];
  kiZA = population[DNAPos].genes[5];

  kdXA = population[DNAPos].genes[6];
  kdYA = population[DNAPos].genes[7];
  kdZA = population[DNAPos].genes[8];


  kpXP = population[DNAPos].genes[9];
  kpYP = population[DNAPos].genes[10];
  kpZP = population[DNAPos].genes[11];

  kiXP = population[DNAPos].genes[12];
  kiYP = population[DNAPos].genes[13];
  kiZP = population[DNAPos].genes[14];

  kdXP = population[DNAPos].genes[15];
  kdYP = population[DNAPos].genes[16];
  kdZP = population[DNAPos].genes[17];

  //millis();
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
  rotateY(thetaYO + 0.785398f);
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

///////////////////////////////////////////////////////////

  pushMatrix();
  translate(500 + displacementXI, yPosition, displacementZI);
  rotateZ(thetaZO);
  rotateX(thetaXO);
  rotateY(thetaYO + 0.785398f);
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

  for (i = 0; i < 200; i++)
  {
    pushMatrix();
    translate(trailPointsX[i], trailPointsY[i], trailPointsZ[i]);
    fill(255,20,147);
    box(2);
    popMatrix();
  }

  float distance = sqrt (((displacementXI-setXP)*(displacementXI-setXP)) + ((displacementZI+setZP)*(displacementZI+setZP)) + ((displacementYI+setYP)*(displacementYI+setYP)));

  timerNew = timerNew + time;

if (distance < 17) {

  //println(DNAPos);
  fitArray[DNAPos] = timerNew;
  DNAPos++;
  //println(timerNew - timerOld + "   ");
  // println(kpXA + "   " + kpYA + "   " + kpZA + "   " + DNAPos);
  displacementXI = 0;
  displacementYI = 0;
  displacementZI = 0;
  displacementXO = 0;
  displacementYO = 0;
  displacementZO = 0;
  velocityXI = 0;
  velocityYI = 0;
  velocityZI = 0;
  velocityXO = 0;
  velocityYO = 0;
  velocityZO = 0;
  integralXA = 0;
  integralYA = 0;
  integralZA = 0;
  integralXP = 0;
  integralYP = 0;
  integralZP = 0;
  thetaZI = 0;
  thetaYI = 0;
  thetaXI = 0;
  thetaZO = 0;
  thetaYO = 0;
  thetaXO = 0;
  omegaIZ = 0;
  omegaIY = 0;
  omegaIX = 0;
  alphaZ = 0;
  alphaY = 0;
  alphaX = 0;
  derivativeXA = 0;
  derivativeYA = 0;
  derivativeZA = 0;
  derivativeXP = 0;
  derivativeYP = 0;
  derivativeZP = 0;
  lastErrorXA = 0;
  lastErrorYA = 0;
  lastErrorZA = 0;
  timerNew = 0;
}

if (DNAPos == population.length){
  generationNo++;
  DNAPos = 0;
  fitArrayMax = max(fitArray);
  fitArrayMin = min(fitArray);

 for(int i=0; i<fitArray.length; i++){
  fitPopulation[i] = fitArrayMax - fitArray[i];
  totalFitPopulation += fitPopulation[i];
  }

 for(int i=0; i<fitArray.length; i++){
  newPopulation[i] = fitPopulation[i]/totalFitPopulation;
  //println(newPopulation[0]);
  }

 //println(totalFitPopulation);
 //println(fitPopulation[0]);
 //println(newPopulation[0]);

 for(int i=0; i<fitArray.length; i++){
  newPopulation[i] = (int)(newPopulation[i]*100);
  totalNewPopulation += newPopulation[i];
  //println(newPopulation[i]);
  println(newPopulation[0]);
  }

      for(int z = 0; z < newPopulation[0]; z++){
      newPopulationFinal[z] = population[0];
      count++;
      println(count + "   1");
    }

      for(int z = 0; z < newPopulation[1]; z++){
      newPopulationFinal[count + z] = population[1];
      count++;
      println(count+ "   2");
    }

      for(int z = 0; z < newPopulation[2]; z++){
      newPopulationFinal[count + z] = population[2];
      count++;
      println(count+ "   3");
    }

      for(int z = 0; z < newPopulation[3]; z++){
      newPopulationFinal[count + z] = population[3];
      count++;
      println(count+ "   4");
    }

      for(int z = 0; z < newPopulation[4]; z++){
      newPopulationFinal[count + z] = population[4];
      count++;
      println(count+ "   5");
    }

      for(int z = 0; z < newPopulation[5]; z++){
      newPopulationFinal[count + z] = population[5];
      count++;
      println(count+ "   6");
    }

      for(int z = 0; z < newPopulation[6]; z++){
      newPopulationFinal[count + z] = population[6];
      count++;
      println(count+ "   7");
    }

      for(int z = 0; z < newPopulation[7]; z++){
      newPopulationFinal[count + z] = population[7];
      count++;
      println(count+ "   8");
    }

      for(int z = 0; z < newPopulation[8]; z++){
      newPopulationFinal[count + z] = population[8];
      count++;
      println(count+ "   9");
    }

      for(int z = 0; z < newPopulation[9]; z++){
      newPopulationFinal[count + z] = population[9];
      count++;
      println(count+ "   10");
    }

count=0;
//println (newPopulationFinal.length);
//println(newPopulationFinal[60]);

  for(int i=0; i<fitArray.length; i++){

  selection1 = (int)random(0,(totalNewPopulation-1));
  //println(selection1);
  //selection2 = (int)random(0,95);
  population[i] = newPopulationFinal[selection1];
  //population[i+1] = newPopulationFinal[selection2];
  //println(totalNewPopulation);
  totalNewPopulation = 0;
  totalFitPopulation = 0;
  //println(fitArrayMin);
  }
  println(fitArrayMax);
  //population[0].genes[0] = (population[0].genes[0] + population[1].genes[0])/2;



  /*

  for(int z=0; z < 8 ; z++){

   for(int i=0; i< 17; i++){
    population[z].genes[i] = (population[z].genes[i] + population[z+1].genes[i])/2;
   }

  }
 */

  /*
  kpXA = (newPopulationFinal[selection1].genes[0] + newPopulationFinal[selection2].genes[0])/2 ;
  kpYA = (newPopulationFinal[selection1].genes[1] + newPopulationFinal[selection2].genes[1])/2 ;
  kpZA = (newPopulationFinal[selection1].genes[2] + newPopulationFinal[selection2].genes[2])/2 ;

  kiXA = (newPopulationFinal[selection1].genes[3] + newPopulationFinal[selection2].genes[3])/2 ;
  kiYA = (newPopulationFinal[selection1].genes[4] + newPopulationFinal[selection2].genes[4])/2 ;
  kiZA = (newPopulationFinal[selection1].genes[5] + newPopulationFinal[selection2].genes[5])/2 ;

  kdXA = (newPopulationFinal[selection1].genes[6] + newPopulationFinal[selection2].genes[6])/2 ;
  kdYA = (newPopulationFinal[selection1].genes[7] + newPopulationFinal[selection2].genes[7])/2 ;
  kdZA = (newPopulationFinal[selection1].genes[8] + newPopulationFinal[selection2].genes[8])/2 ;


  kpXP = (newPopulationFinal[selection1].genes[9] + newPopulationFinal[selection2].genes[9])/2 ;
  kpYP = (newPopulationFinal[selection1].genes[10] + newPopulationFinal[selection2].genes[10])/2 ;
  kpZP = (newPopulationFinal[selection1].genes[11] + newPopulationFinal[selection2].genes[11])/2 ;

  kiXP = (newPopulationFinal[selection1].genes[12] + newPopulationFinal[selection2].genes[12])/2 ;
  kiYP = (newPopulationFinal[selection1].genes[13] + newPopulationFinal[selection2].genes[13])/2 ;
  kiZP = (newPopulationFinal[selection1].genes[14] + newPopulationFinal[selection2].genes[14])/2 ;

  kdXP = (newPopulationFinal[selection1].genes[15] + newPopulationFinal[selection2].genes[15])/2 ;
  kdYP = (newPopulationFinal[selection1].genes[16] + newPopulationFinal[selection2].genes[16])/2 ;
  kdZP = (newPopulationFinal[selection1].genes[17] + newPopulationFinal[selection2].genes[17])/2 ;
  println(totalNewPopulation);

  }
  */

 }

  //println(displacementXI + "    " + displacementYI + "    " + displacementZI + "    " + frameRate);
  //println(velocityXI + "    " + velocityYI + "    " + velocityZI + "    " + frameRate);
  //println(liftX + "    " + liftZ + "    " + dragY + "    " + frameRate);
  //println( dragX + "   " + dragZ + "   " + liftX + "   " + liftZ);

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
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////



public void calculateZAngle() {

  // : 2/3 since mass of both arms is included and multiplied by sin(0.785398) to account for length shortening in quadX configuration

  alphaZ = ((motor3Thrust+motor4Thrust) - (motor2Thrust+motor1Thrust))  / (((2/3)*armMass + 2*motorMass)*(armLength*sin(0.785398f)));

  omegaIZ = omegaIZ + (alphaZ * time);

  thetaZO = thetaZI + omegaIZ*time + (1/2)*alphaZ*time*time;

  thetaZI = thetaZO;

}

public void calculateXAngle() {

  alphaX = ((motor1Thrust+motor3Thrust) - (motor4Thrust+motor2Thrust) ) / (((2/3)*armMass + 2*motorMass)*armLength*sin(0.785398f));

  omegaIX = omegaIX + (alphaX * time);

  thetaXO = thetaXI + omegaIX*time + (1/2)*alphaX*time*time;

  thetaXI = thetaXO;

}

public void calculateYAngle() {

  alphaY = ((motor4Thrust+motor1Thrust)*torqueFactor - (motor3Thrust+motor2Thrust)*torqueFactor) / (((1/3)*(armMass*2+motorMass*2)*(2*armLength)*(2*armLength))+((armMass*2+motorMass*2)*armLength*armLength));

  omegaIY = omegaIY + (alphaY * time);

  thetaYO = thetaYI + omegaIY*time + (1/2)*alphaY*time*time;

  thetaYI = thetaYO;

}



///////////////////////////////////////////////////////////



public void movementX() {

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
public void movementY() {

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


public void movementZ() {

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
public void pidAngleX(){

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


public void pidAngleZ(){

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

public void pidAngleY(){

 errorYA = setYA - thetaYO;

 integralYA = integralYA + errorYA*time;

 derivativeYA = (errorYA-lastErrorYA)/time;

 pYAError = kpYA*errorYA + kiYA*integralYA + kdYA*derivativeYA;

 lastErrorYA = errorYA;

}

/////////////////////////////////////////////////////////////////////////////////

////PID POSITION CONTROLLER//////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

public void pidPositionY(){

 errorYP = setYP + displacementYI;

 integralYP = integralYP + (errorYP*time);

 derivativeYP = (errorYP-lastErrorYP)/time;

 pYError = (kpYP*errorYP + kiYP*integralYP + kdYP*derivativeYP);

 lastErrorYP = errorYP;

}

public void pidPositionX(){

 errorXP = setXP - displacementXI;

 integralXP = integralXP + errorXP*time;

 derivativeXP = (errorXP-lastErrorXP)/time;

 pXError = (kpXP*errorXP + kiXP*integralXP + kdXP*derivativeXP);

 lastErrorXP = errorXP;

}

public void pidPositionZ(){

 errorZP = -(setZP + displacementZI);

 integralZP = integralZP + errorZP*time;

 derivativeZP = (errorZP-lastErrorZP)/time;

 pZError = kpZP*errorZP + kiZP*integralZP + kdZP*derivativeZP;

 lastErrorZP = errorZP;

}

///////////////////////////////////////////////////////////

public void motorCalculations(){

  motor1Thrust = pYError  - pXAError + pZAError + pYAError;

  motor2Thrust = pYError  + pXAError + pZAError - pYAError;

  motor3Thrust = pYError  - pXAError - pZAError - pYAError;

  motor4Thrust = pYError  + pXAError - pZAError + pYAError;

}
///////////////////////////////////////////////////////////

public void motorEndpoints(){

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

public void dragCalculations(){

  if (velocityXI > 0){
  cDX = 1.28f * sqrt(sin(thetaZO) * sin(thetaZO));
  dragX = -(0.5f * cDX * density * velocityXI * velocityXI * (surfaceA*sqrt(sin(thetaZO)*sin(thetaZO))));
  }
  else{
  cDX = 1.28f * sqrt(sin(thetaZO) * sin(thetaZO));
  dragX = (0.5f * cDX * density * velocityXI * velocityXI * (surfaceA*sqrt(sin(thetaZO)*sin(thetaZO))));
  }

  if (velocityZI > 0){
  cDZ = 1.28f * sqrt(sin(thetaXO) * sin(thetaXO));
  dragZ = -(0.5f * cDZ * density * velocityZI * velocityZI * (surfaceA*sqrt(sin(thetaXO)*sin(thetaXO))));
  }
  else{
  cDZ = 1.28f * sqrt(sin(thetaXO) * sin(thetaXO));
  dragZ = (0.5f * cDZ * density * velocityZI * velocityZI * (surfaceA*sqrt(sin(thetaXO)*sin(thetaXO))));
  }

  if (velocityYI > 0){
  cDY = ((1.28f * cos(thetaXO)) + (1.28f * cos(thetaZO)))/2 ;
  dragY = (0.5f * cDY * density * velocityYI * velocityYI * (surfaceA*sqrt(cos(thetaXO)*cos(thetaXO))*sqrt(cos(thetaZO)*cos(thetaZO))));
  }
  else{
  cDY = ((1.28f * cos(thetaXO)) + (1.28f * cos(thetaZO)))/2 ;
  dragY = -(0.5f * cDY * density * velocityYI * velocityYI * (surfaceA*sqrt(cos(thetaXO)*cos(thetaXO))*sqrt(cos(thetaZO)*cos(thetaZO))));
  }

}

public void liftCalculations(){

  if (thetaZO > 0 && velocityXI > 0){
  cLX = 2 * PI * sqrt(thetaZO*thetaZO);
  liftX = (0.5f * cLX * density * velocityXI * velocityXI * (surfaceA*sqrt(cos(thetaZO)*cos(thetaZO))));
  }

  if (thetaZO < 0 && velocityXI > 0){
  cLX = 2* PI * sqrt(thetaZO*thetaZO);
  liftX = -(0.5f * cLX * density * velocityXI * velocityXI * (surfaceA*sqrt(cos(thetaZO)*cos(thetaZO))));
  }

  if (thetaZO > 0 && velocityXI < 0){
  cLX = 2* PI * sqrt(thetaZO*thetaZO);
  liftX = -(0.5f * cLX * density * velocityXI * velocityXI * (surfaceA*sqrt(cos(thetaZO)*cos(thetaZO))));
  }

  if (thetaZO < 0 && velocityXI < 0){
  cLX = 2* PI * sqrt(thetaZO*thetaZO);
  liftX = (0.5f * cLX * density * velocityXI * velocityXI * (surfaceA*sqrt(cos(thetaZO)*cos(thetaZO))));
  }



  if (thetaXO > 0 && velocityZI > 0){
  cLX = 2 * PI * sqrt(thetaXO*thetaXO);
  liftZ = (0.5f * cLX * density * velocityZI * velocityZI * (surfaceA*sqrt(cos(thetaXO)*cos(thetaXO))));
  }

  if (thetaXO < 0 && velocityZI > 0){
  cLX = 2* PI * sqrt(thetaXO*thetaXO);
  liftZ = -(0.5f * cLX * density * velocityZI * velocityZI * (surfaceA*sqrt(cos(thetaXO)*cos(thetaXO))));
  }

  if (thetaXO > 0 && velocityZI < 0){
  cLX = 2* PI * sqrt(thetaXO*thetaXO);
  liftZ = -(0.5f * cLX * density * velocityZI * velocityZI * (surfaceA*sqrt(cos(thetaXO)*cos(thetaXO))));
  }

  if (thetaXO < 0 && velocityZI < 0){
  cLX = 2* PI * sqrt(thetaXO*thetaXO);
  liftZ = (0.5f * cLX * density * velocityZI * velocityZI * (surfaceA*sqrt(cos(thetaXO)*cos(thetaXO))));
  }

}
class DNA {

  float[] genes = new float[18];


  DNA() {

  float genesPAXZ;  //genes Proportional Angle Xdirection Zdirection
  float genesIAXZ;
  float genesDAXZ;
  float genesPAY;
  float genesIAY;
  float genesDAY;

  float genesPPXZ;  //genes Proportional Position Xdirection Zdirection
  float genesIPXZ;
  float genesDPXZ;
  float genesPPY;
  float genesIPY;
  float genesDPY;

////////////xzAngle///////////////

      genesPAXZ= random(0, 100);
      genesPAXZ = genesPAXZ/100000;

      genesIAXZ = random(0, 101);
      genesIAXZ = genesIAXZ/10000000;

      genesDAXZ = random(0, 101);
      genesDAXZ = genesDAXZ/10000;

////////////yAngle/////////////////

      genesPAY = random(0, 101);
      genesPAY = genesPAY/100000;

      genesIAY = random(0, 101);
      genesIAY = genesIAY/10000000;

      genesDAY = random(0, 101);
      genesDAY = genesDAY/10000;

////////////xzPosition///////////////

      genesPPXZ = random(0, 101);
      genesPPXZ = genesPPXZ/100;

      genesIPXZ = random(0, 101);
      genesIPXZ = genesIPXZ/100000;

      genesDPXZ = random(0, 101);
      genesDPXZ = genesDPXZ/100;

////////////yPosition/////////////////

      genesPPY = random(0, 101);
      genesPPY = genesPPY/100;

      genesIPY = random(0, 101);
      genesIPY = genesIPY/10000;

      genesDPY = random(0, 101);
      genesDPY = genesDPY/10;



  genes[0] = kpXA + genesPAXZ;
  genes[1] = kpYA + genesPAY;
  genes[2] = kpZA + genesPAXZ;

  genes[3] = kiXA + genesIAXZ;
  genes[4] = kiYA + genesIAY;
  genes[5] = kiZA + genesIAXZ;

  genes[6] = kdXA + genesDAXZ;
  genes[7] = kdYA + genesDAY;
  genes[8] = kdZA + genesDAXZ;


  genes[9] = kpXP + genesPPXZ;
  genes[10] = kpYP + genesPPY;
  genes[11] = kpZP + genesPPXZ;

  genes[12] = kiXP + genesIPXZ;
  genes[13] = kiYP + genesIPY;
  genes[14] = kiZP + genesIPXZ;

  genes[15] = kdXP + genesDPXZ;
  genes[16] = kdYP + genesDPY;
  genes[17] = kdZP + genesDPXZ;

  }
}
class Route {
  
  float[] xPoint = new float[10];
  float[] zPoint = new float[10];
  float[] yPoint = new float[10]; 
  
  Route(){
    
  
  xPoint[0] = 0;
  xPoint[1] = 0;
  xPoint[2] = -200;
  xPoint[3] = -200;
  xPoint[4] = -100;
  xPoint[5] = -100;
  xPoint[6] = -170;
  xPoint[7] = -80;
  xPoint[8] = -100;
  xPoint[9] = 0;
  
  zPoint[0] = 0;
  zPoint[1] = 200;
  zPoint[2] = 200;
  zPoint[3] = -100;
  zPoint[4] = -100;
  zPoint[5] = 0;
  zPoint[6] = 30;
  zPoint[7] = 60;
  zPoint[8] = 0;
  zPoint[9] = 0;
  
  yPoint[0] = 200;
  yPoint[1] = 200;
  yPoint[2] = 200;
  yPoint[3] = 200;
  yPoint[4] = 200;
  yPoint[5] = 200;
  yPoint[6] = 200;
  yPoint[7] = 200;
  yPoint[8] = 200;
  yPoint[9] = 200;
  
}

}
  public void settings() {  size(1000, 700, P3D); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "XQuad" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
