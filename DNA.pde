class DNA {
  
  float[] genes = new float[18];  
  
  float[] genesPAXZ = new float[10];  //genes Proportional Angle Xdirection Zdirection
  float[] genesIAXZ = new float[10];
  float[] genesDAXZ = new float[10];
  float[] genesPAY = new float[10];
  float[] genesIAY = new float[10];
  float[] genesDAY = new float[10];
  
  float[] genesPPXZ = new float[10];  //genes Proportional Position Xdirection Zdirection
  float[] genesIPXZ = new float[10];
  float[] genesDPXZ = new float[10];
  float[] genesPPY = new float[10];
  float[] genesIPY = new float[10];
  float[] genesDPY = new float[10];
  
  
  DNA() {
    
////////////xzAngle///////////////

    for (int i = 0; i < genesPAXZ.length; i++) {
      genesPAXZ[i] = random(0.0001, -0.0001);
    }
    
    for (int i = 0; i < genesIAXZ.length; i++) {
      genesIAXZ[i] = random(0.000001, -0.000001);
    }
    
    for (int i = 0; i < genesDAXZ.length; i++) {
      genesDAXZ[i] = random(0.001, -0.001);
    }
////////////yAngle/////////////////    
      
    for (int i = 0; i < genesPAY.length; i++) {
      genesPAY[i] = random(0.0001, -0.0001);
    }
    
    for (int i = 0; i < genesIAY.length; i++) {
      genesIAY[i] = random(0.000001, -0.000001);
    }
    
    for (int i = 0; i < genesDAY.length; i++) {
      genesDAY[i] = random(0.001, -0.001);
    }
    
    
    
////////////xzPosition///////////////

    for (int i = 0; i < genesPPXZ.length; i++) {
      genesPPXZ[i] = random(0.1, -0.1);
    }
    
    for (int i = 0; i < genesIPXZ.length; i++) {
      genesIPXZ[i] = random(0.0001, -0.0001);
    }
    
    for (int i = 0; i < genesDPXZ.length; i++) {
      genesDPXZ[i] = random(0.1, -0.1);
    }
////////////yPosition/////////////////   000 
      
    for (int i = 0; i < genesPPY.length; i++) {
      genesPPY[i] = random(0.1, -0.1);
    }
    
    for (int i = 0; i < genesIPY.length; i++) {
      genesIPY[i] = random(0.001, -0.001);
    }
    
    for (int i = 0; i < genesDPY.length; i++) {
      genesDPY[i] = random(1, -1);
    }   
    
    
    
    
  }
  
  void DNASetup(){
   
  genes[0] = kpXA + genesPAXZ[i]; 
  genes[1] = kpYA + genesPAY[i];
  genes[2] = kpZA + genesPAXZ[i];

  genes[3] = kiXA + genesIAXZ[i];
  genes[4] = kiYA + genesIAY[i];
  genes[5] = kiZA + genesIAXZ[i];

  genes[6] = kdXA + genesDAXZ[i];
  genes[7] = kdYA + genesDAY[i];
  genes[8] = kdZA + genesDAXZ[i];
  
  
  genes[9] = kpXP + genesPPXZ[i];
  genes[10] = kpYP + genesPPY[i];
  genes[11] = kpZP + genesPPXZ[i];

  genes[12] = kiXP + genesIPXZ[i];
  genes[13] = kiYP + genesIPY[i];
  genes[14] = kiZP + genesIPXZ[i];

  genes[15] = kdXP + genesDPXZ[i];
  genes[16] = kdYP + genesDPY[i];
  genes[17] = kdZP + genesDPXZ[i];
   
  }
  
}