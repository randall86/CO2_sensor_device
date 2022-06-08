#include <u600.h>
#include <curveFitting.h>
#include <MovingAverageFilter.h>

#define SERIAL_ENABLED_TRUE

MovingAverageFilter movingAverageFilter(180); //Use 180 data points for moving average 

//-------------------------------------------------------------------------------
///////////////////////////////// U600 ///////////////////////////////////////////
U600 co2Sensor(Serial2, 100);
//U600 co2Sensor(Serial2, 100, 22, 23); //SENSOR_RXD - 22, SENSOR_TXD - 23

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
  co2Sensor.recalibration();
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
  #ifdef U600_RAW_DATA_LOG
  if (Serial2.available() > 0) {
    Serial.println(Serial2.read(), HEX);
  }
  #endif
}

void updateCo2(co2_t* co2Packet){
  co2 = co2Sensor.getCo2Concentration(co2Packet);

  z = 0.5335 * z + 0.4665 * co2;
  co2Avg = movingAverageFilter.process(z);
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
  extractS1Parameter();
  extractS6Parameter();
  extractS1xS6Angle();
  extractHjorthActivity();
    
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


void extractHjorthActivity(){
  double co2Average = co2CumulativeSum/capnoIndex;
  double co2DiffSquaredCumulativeSum = 0.0;
  for(int i=0; i<capnoIndex; i++ ){
    co2DiffSquaredCumulativeSum += sq(capnogram[i]-co2Average);
  }
  hjorthActivity = co2DiffSquaredCumulativeSum/capnoIndex;
  hjorthActivity = sqrt(hjorthActivity);
}

void extractS1Parameter(){ 
  fitCurve(1, s1EndIndex+1, timeAxis, &capnogram[0], 2, s1Coefficients);
}

void extractS6Parameter(){
  s6StartIndex = 0; 
  for(int i=capturedEtCo2Index; i<capnoIndex; i++){
    if(capnogram[i] <= S6_START){
      s6StartIndex = i;
      break;     
    } 
  }
  uint16_t s6DataPointLength = capnoIndex - s6StartIndex;
  fitCurve(1, s6DataPointLength, &timeAxis[s6StartIndex], &capnogram[s6StartIndex], 2, s6Coefficients);
}

void extractS1xS6Angle(){
  double m1 = s1Coefficients[1];
  double m2 = s6Coefficients[1];
  double tanTheta =  (m2-m1)/(1+(m1*m2));
  S3rad = atan(tanTheta);
  S3deg = (abs(S3rad)*180)/PI;
  if(S3rad < 0){
    S3deg = 180 - S3deg;
  }
}

void showProcessedSignal(){
  for(int i=0; i<capnoIndex; i++ ){
    // Complete Capnogram
    Serial.print(capnogram[i]); 
    Serial.print(",");
    
    //S1 section
    if(i<= s1EndIndex)
      Serial.print(capnogram[i]); // S1 signal
    else
      Serial.print(0.0);
    
    Serial.print(",");    
    Serial.print((s1Coefficients[1]*(timeAxis[i]))+s1Coefficients[0]); // S1 Linear regression y=mx+c

    //S6 section
    Serial.print(","); 
    if(i>= s6StartIndex)
      Serial.print(capnogram[i]); // S6 signal 
    else{
      Serial.print(0.0);
    }
    Serial.print(",");  
    Serial.print((s6Coefficients[1]*(timeAxis[i]))+s6Coefficients[0]); // S6 Linear regression y=mx+c  

    Serial.print(",");
    Serial.print(S3deg);

    Serial.println("");
  }
}
