
#include "u600.h"
#include <MovingAverageFilter.h>
//-------------------------------------------------------------------------------
///////////////////////////////// U600 ///////////////////////////////////////////
U600 co2Sensor(Serial2,100);

float co2 = -0.001;
float etCo2 = -0.001;
float respirationRate = -0.001;
float inspiration = -0.001;

// Hjorth Activity Parameters
float z;
float co2Avg = 0.0;
double co2CumulativeSum = 0.0;

float hjorthActivity = 0.0;

// Asthma Classification
int result0 = 0;
int result1 = 0;
int result2 = 0;
int result3 = 0;
int data0 = 0;  
int data1 = 0; 
int data2 = 0;
int data3 = 0;
int asthma = 0;

const int MAX_CAPNO_LENGTH = 300; // 300 datapoints for a maximum of 3 seconds duration of capnograms
double capnogram[MAX_CAPNO_LENGTH];           
double timeAxis[MAX_CAPNO_LENGTH];

double capturedEtCo2  = 0.0;
uint16_t capturedEtCo2Index = 0;
uint16_t capnoIndex  = 0;       // current datapoint index
boolean capnoCapture = false;   // start capno capture flag

// Slope 1 parameters on inspiration
// - start when co2 >= 4 mmHg (the start of capnogram) and end at co2>= 11
double S1_STOP  = 11;
double s1Coefficients[2]; // y=mx+c : m = s1Coefficients[1] c = s1Coefficients[0]

uint16_t s1EndIndex = 0;
boolean s1EndDetected = false;

// Slope 6 parameters on respiration (after etCo2)
// - start when co2 <= 10 mmHg and end at co2 <= 4 mmHg
double S6_START = 10;
double s6Coefficients[2]; // y=mx+c : m = s1Coefficients[1] c = s1Coefficients[0]

uint16_t s6StartIndex = 0;

// Angle between S1 and S6
double S3rad = 0.0;
double S3deg = 0.0;
//----------------------------------------------------------------------------


void setup() {

  Serial.begin(115200);
  Serial.println("System initialization ...");

  co2Sensor.init();

  #ifdef U600_RECALIBRATION_TRUE
  u600_recalibration();
  #endif

  Serial.println("... Initialization completed.");

}

void loop() {
  co2Sensor.read();
  Serial.println("... reading.");
  while(co2Sensor.isAvailable()){
    co2_t* co2Packet = co2Sensor.getCo2Reading();
    if(co2Packet!=NULL){
      Serial.println("... extracting data.");
      updateCo2(co2Packet);
      updateEtCo2(co2Packet);
      updateRespirationRate(co2Packet);
      updateInspiration(co2Packet);
      updateAsthmaSeverity();

    }
  }
}

void updateCo2(co2_t* co2Packet){
  co2 = co2Sensor.getCo2Concentration(co2Packet);

  z = 0.5335 * z + 0.4665 * co2;
  //co2Avg = movingAverageFilter.process(z);
  co2Avg = z;
  Serial.println("co2Avg :");
  Serial.println(co2Avg);
  assembleCapnogram(co2Avg);

}

void updateEtCo2(co2_t* co2Packet){
  if(co2Sensor.isEtCo2(co2Packet)){
    etCo2 = co2Sensor.getEtCo2(co2Packet);
    Serial.println("etCo2 :");
    Serial.println(etCo2);
  }
}

void updateRespirationRate(co2_t* co2Packet){
  if(co2Sensor.isRespirationRate(co2Packet)){
    respirationRate = co2Sensor.getRespirationRate(co2Packet);
    Serial.println("respirationRate :");
    Serial.println(respirationRate);
  }
}

void updateInspiration(co2_t* co2Packet){
  if(co2Sensor.isInspiration(co2Packet)){
    inspiration = co2Sensor.getInspiration(co2Packet);
    Serial.println("inspiration :");
    Serial.println(inspiration);
  }
}

void updateAsthmaSeverity(){
  //  //Turns Count algorithm for classification of asthma severity
  if ((etCo2 >= 32) && (etCo2 <= 45) && (respirationRate > 10) && (respirationRate < 30)){
    result1 = result1 + 1;
    result0 = 0;
    result2 = 0;
    result3 = 0;
  }

  if ((data1 == 1) && (etCo2 == 0) && (respirationRate == 0)){
    data1 = 0;
  }

  if ((result1 == 20) && (data1 == 0)){
    asthma = 1;
    result1 = 0;
    data1 = 1;
  }


  if ((etCo2 < 32) && (respirationRate > 28)){
    result2 = result2 + 1;
    result0 = 0;
    result1 = 0;
    result3 = 0;
  }

  if ((data2 == 1) && (etCo2 == 0) && (respirationRate == 0)){
    data2 = 0;
  }

  if ((result2 == 25) && (data2 == 0)){
    asthma = 5;
    result2 = 0;
    data2 = 1;
  }
  
  if ((etCo2 >= 45 ) && (respirationRate < 8)){
    result0 = result0 + 1;
    result1 = 0;
    result2 = 0;
    result3 = 0;
  }

  if ((data0 == 1) && (etCo2 == 0) && (respirationRate == 0)){
    data0 = 0;
  }

  if ((result0 == 8) && (data0 == 0)){
    asthma = 10;
    result0 = 0;
    data0 = 1;
  }

  if ((etCo2 <= 5 ) ||  (respirationRate <= 5)){
    result3 = result3 + 1;
    result0 = 0;
    result1 = 0;
    result2 = 0;
  }

  if ((data3 == 1) && (etCo2 == 0) && (respirationRate == 0)){
    data3 = 0;
  }

  if ((result3 == 4) && (data3 == 0)){
    asthma = 0;
    result3 = 0;
    data3 = 1;
  }
}

void assembleCapnogram(float co2){
  if(co2 >= 4.0){
    // assign the start of capno to be >= 4 mmHg
    if(!capnoCapture){
      capnoCapture = true;
      co2CumulativeSum = 0.0;
      capnoIndex = 0;
    }
    if(capnoIndex > 299){
      // the duration of the capno is longer than the pre-allocate buffer
      return;
    } 
    capnogram[capnoIndex] = co2;
    co2CumulativeSum += co2;
    
    // Update etCo2 index and value
    if(co2 >= capturedEtCo2){
      capturedEtCo2 = co2;
      capturedEtCo2Index = capnoIndex;
    }

    // S1 detector
    if(!s1EndDetected && co2 >= S1_STOP){
      s1EndIndex = capnoIndex;
      s1EndDetected = true;
    }

    capnoIndex++;
    return;
  }

  // co2 level dips below 4mmHg (assumes to be the end of the capnogram)
  if(capnoCapture){
    featureExtraction();

    // reset the parameters
    capnoCapture = false;
    capnoIndex = 0;
    capturedEtCo2 = 0.0;
    capturedEtCo2Index = 0;
    s1EndIndex = 0;
    s1EndDetected = false;
  }
}

void featureExtraction(){
  // process the capnogram
  //extractS1Parameter();
  //extractS6Parameter();
  //extractS1xS6Angle();
  //extractHjorthActivity();
    
  #ifdef SERIAL_ENABLED_TRUE
  showProcessedSignal();
  #endif  

  //#ifdef SERIAL_ENABLED_FALSE
  Serial.print("HA: ");
  Serial.print(hjorthActivity);
  Serial.print(" ,S1 slope (m1): ");
  Serial.print(s1Coefficients[1]);
  Serial.print(" ,S6 slope (m2): ");
  Serial.print(s6Coefficients[1]);
  Serial.print(" ,S3 radian : ");
  Serial.print(S3rad);
  Serial.print(" ,S3 degree : ");
  Serial.println(S3deg);
  //#endif 
}
