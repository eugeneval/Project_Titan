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
