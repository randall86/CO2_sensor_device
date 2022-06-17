// I-Breath CO2 Device
// Rev 0.1 (16/06/2022)
// - Infinecs

#include <Ticker.h>
#include <FS.h>
#include <SD_MMC.h>
#include <u600.h>
#include <curveFitting.h>
#include <MovingAverageFilter.h>

//#define SDCARD_INIT
//#define SHOW_PROC_SIGNAL

const char * app_ver = "v0.1";

MovingAverageFilter movingAverageFilter(180); //Use 180 data points for moving average

//GPIO pins
const byte VBAT_ADC = 18;
const byte SENSOR_RXD = 22;
const byte SENSOR_TXD = 23;
const byte BUTTON_1 = 26;
const byte BUTTON_2 = 27;
const byte LCD_RX2 = 33;
const byte LCD_TX2 = 35;

const char DELIM = ',';
const char NEWLINE = '\n';

//-------------------------------------------------------------------------------
///////////////////////////////// Buttons ///////////////////////////////////////
const int BTN_DEBOUNCE_MS = 50;
const int BTN_CHECK_MS = 10;
static byte buttonPin[2] = {BUTTON_1, BUTTON_2};
static byte debouncedBtnState[2] = {1, 1};
static bool buttonPressed[2] = {false, false};
static uint8_t buttonCount[2] = {BTN_DEBOUNCE_MS/BTN_CHECK_MS, BTN_DEBOUNCE_MS/BTN_CHECK_MS};
Ticker button1Ticker;
Ticker button2Ticker;

//-------------------------------------------------------------------------------
///////////////////////////////// U600 ///////////////////////////////////////////
U600 co2Sensor(Serial2, 100, SENSOR_RXD, SENSOR_TXD);

float co2 = -0.001;
float etCo2 = -0.001;
float respirationRate = -0.001;
float inspiration = -0.001;

// Hjorth Activity Parameters
float z;
float co2Avg = 0.0;
double co2CumulativeSum = 0.0;

float hjorthActivity = 0.0;

const int MAX_CAPNO_LENGTH = 300; // 300 datapoints for a maximum of 3 seconds duration of capnograms
double capnogram[MAX_CAPNO_LENGTH];                     
double timeAxis[MAX_CAPNO_LENGTH];

double capturedEtCo2    = 0.0;
uint16_t capturedEtCo2Index = 0;
uint16_t capnoIndex    = 0;             // current datapoint index
boolean capnoCapture = false;     // start capno capture flag

// Slope 1 parameters on inspiration
// - start when co2 >= 4 mmHg (the start of capnogram) and end at co2>= 11
double S1_STOP    = 11;
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

//-------------------------------------------------------------------------------
///////////////////////////////// SD CARD ///////////////////////////////////////
const char * loggingFile = "/data.log";
boolean sdCardAvailable = false;
boolean loggingIsOn = false;
static char patient[32] = "";
//----------------------------------------------------------------------------

boolean isLoggingOn(){
    static boolean current_logging_state = false;
    if(buttonPressed[0]){
        buttonPressed[0] = false; //reset the button state
        current_logging_state = !current_logging_state;
        
        if(current_logging_state){
            Serial.println("Logging started ....");
            String header = "";

            if(strlen(patient)){
                header = String("PATIENT") + String(DELIM) + 
                        String("AVGCO2") + String(DELIM) +
                        String("ETCO2") + String(DELIM) +
                        String("RR") + String(DELIM) +
                        String("AVGACTCO2") + String(DELIM) +
                        String("S3(degree)") + String(NEWLINE);
            }
            else{
                header = String("AVGCO2") + String(DELIM) +
                        String("ETCO2") + String(DELIM) +
                        String("RR") + String(DELIM) +
                        String("AVGACTCO2") + String(DELIM) +
                        String("S3(degree)") + String(NEWLINE);
            }

            Serial.print(header.c_str());
            if(sdCardAvailable){
                appendFile(loggingFile, const_cast<char *>(header.c_str()));
            }
        }
        else{
            Serial.println("Logging stopped ....");
        }
    }
    return current_logging_state;
}

void appendFile(const char * path, const char * data){
    File file = SD_MMC.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(!file.println(data)){
        Serial.println("Append failed");
    }
    file.close();
}

void deleteFileIfAvailable(const char * path){
    File file = SD_MMC.open(path);
    if(file){
        //only delete if file is available
        if(file.available())
        {
            file.close();
            if(!SD_MMC.remove(path)){
                Serial.println("Delete failed");
            } 
        }
    }
}

//returns true if state changed
bool debounceBtnSw(byte *state, int button_num){
    bool state_changed = false;

    //read the door switch from the HW
    byte raw_state = digitalRead(buttonPin[button_num]);
    *state = debouncedBtnState[button_num];

    if (raw_state == debouncedBtnState[button_num])
    {
        //set the timer which allows a change from current state.
        buttonCount[button_num] = BTN_DEBOUNCE_MS/BTN_CHECK_MS;
    }
    else
    {
        //state has changed - wait for new state to become stable.
        if (--buttonCount[button_num] == 0)
        {
            // Timer expired - accept the change.
            debouncedBtnState[button_num] = raw_state;
            state_changed = true;
            *state = debouncedBtnState[button_num];

            // And reset the timer.
            buttonCount[button_num] = BTN_DEBOUNCE_MS/BTN_CHECK_MS;
        }
    }

    return state_changed;
}

void debounceBtnSWRoutine(int button_num){
    byte switch_state = 0;

    if(debounceBtnSw(&switch_state, button_num))
    {
        if(!switch_state)
        {
            buttonPressed[button_num] = true;
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.print("I-Breath CO2 Device ");
    Serial.println(app_ver);
    Serial.println("System initialization ...");
    
    for(int i=0; i< MAX_CAPNO_LENGTH; i++){
        timeAxis[i] = i;
    }

    pinMode(buttonPin[0], INPUT);
    pinMode(buttonPin[1], INPUT);

    Serial.println("... Initializing CO2 sensor.");
    co2Sensor.init();

    #ifdef U600_RECALIBRATION_TRUE
    co2Sensor.recalibration();
    #endif

    #ifdef SDCARD_INIT
    if(SD_MMC.begin()){
        if(SD_MMC.cardType() != CARD_NONE)
        {
            Serial.println("... SD card is available.");
            sdCardAvailable = true;
        }
    }
    #endif

    Serial.println("... Initializing button tickers");
    button1Ticker.attach_ms(BTN_CHECK_MS, debounceBtnSWRoutine, 0);
    button2Ticker.attach_ms(BTN_CHECK_MS, debounceBtnSWRoutine, 1);

    Serial.println("... Initialization completed. Push button to start logging.");
}

void loop() {
    loggingIsOn = isLoggingOn();
    co2Sensor.read();
    //Serial.println("... reading.");
    while(co2Sensor.isAvailable()){
        co2_t* co2Packet = co2Sensor.getCo2Reading();
        if(co2Packet!=NULL){
            //Serial.println("... extracting data.");
            updateCo2(co2Packet);
            updateEtCo2(co2Packet);
            updateRespirationRate(co2Packet);
            updateInspiration(co2Packet);
            
            doLogging();
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
    assembleCapnogram(co2Avg);
}

void updateEtCo2(co2_t* co2Packet){
    if(co2Sensor.isEtCo2(co2Packet)){
        etCo2 = co2Sensor.getEtCo2(co2Packet);
    }
}

void updateRespirationRate(co2_t* co2Packet){
    if(co2Sensor.isRespirationRate(co2Packet)){
        respirationRate = co2Sensor.getRespirationRate(co2Packet);
    }
}

void updateInspiration(co2_t* co2Packet){
    if(co2Sensor.isInspiration(co2Packet)){
        inspiration = co2Sensor.getInspiration(co2Packet);
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
        
    #ifdef SHOW_PROC_SIGNAL
    showProcessedSignal();
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
    #endif 
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
    double tanTheta =    (m2-m1)/(1+(m1*m2));
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

void doLogging(){
    if(loggingIsOn){
        String log = "";
        String co2_data = String(co2Avg) + String(DELIM) +
                        String(etCo2) + String(DELIM) +
                        String(respirationRate) + String(DELIM) +
                        String(hjorthActivity) + String(DELIM) +
                        String(S3deg) + String(NEWLINE);
        //prepend patient name if available
        if(strlen(patient)){
            log = String(patient) + String(DELIM) + co2_data;
        }
        else{
            log = co2_data;
        }
        Serial.print(log.c_str());
        if(sdCardAvailable){
            appendFile(loggingFile, const_cast<char *>(log.c_str()));
        }
    }
}
