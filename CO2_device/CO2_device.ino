// I-Breath CO2 Device
// Rev 1.3 (23/07/2022)
// - Infinecs
//Serial - UART serial monitor
//Serial1 - LCD 
//Serial2 - CO2 sensor

#include <Ticker.h>
#include <FS.h>
#include <SD_MMC.h>
#include <u600.h>
#include <curveFitting.h>
#include <MovingAverageFilter.h>
#include <ESP32Time.h>
#include <Wire.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

TaskHandle_t serialTask;

//Temporary LCD write time  20 Sept 2022 , 03:45:00 AM
//byte TIME[]={0x5A,0xA5,0x0B,0x82,0x00,0x9C,0x5A,0xA5,0x16,0x09,0x14,0x03,0x2D,0x00};


//#define DEBUG_LCD
//#define DEBUG_CO2
#define SDCARD_INIT
//#define SHOW_PROC_SIGNAL

#define LOW_BYTE(x)     ((byte)((x)&0xFF))
#define HIGH_BYTE(x)    ((byte)(((x)>>8)&0xFF))

//RIMS
#define MEASURING 0
#define NORMAL 1
#define MILD 2
#define SEVERE 3

const char * app_ver = "v1.3";

MovingAverageFilter movingAverageFilter(180); //Use 180 data points for moving average

//GPIO pins
const byte USER_LED1 = 5;
const byte VBAT_ADC = 36;
const byte SENSOR_RXD = 22;
const byte SENSOR_TXD = 23;
const byte BUTTON_1 = 26;
const byte BUTTON_2 = 27;
const byte LCD_RX2 = 35; //swapped from schematics
const byte LCD_TX2 = 33; //swapped from schematics

const char DELIM = ',';
//NOTE: originally it was \n for CR but due to sometimes DWIN LCD returning \n in the middle of RTC reply , it causes issues like cannot read RTC time. \r not used by LCD.
const char NEWLINE = '\r';  


//-------------------------------------------------------------------------------
/////////////////////////////////// ADC /////////////////////////////////////////
#define R2 100  //voltage divider resistor
#define R3 100  //voltage divider resistor
#define VOLTAGE_MAX 4200
#define VOLTAGE_MIN 3300
#define VOLTAGE_OUT(Vin) (((Vin) * R3) / (R2 + R3))  //2100
#define ADC_REFERENCE 1100
#define VOLTAGE_TO_ADC(in) ((ADC_REFERENCE * (in)) / 4096)  //563
#define BATTERY_MAX_ADC VOLTAGE_TO_ADC(VOLTAGE_OUT(VOLTAGE_MAX))
#define BATTERY_MIN_ADC VOLTAGE_TO_ADC(VOLTAGE_OUT(VOLTAGE_MIN))
//averaging for ADC level. NOTE: higher number better stability at the cost of performance. This will impact batt levels and also connection/response state.
#define FILTER_LEN  40

uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0};
int AN_Pot1_i = 0;
int AN_Pot1_Raw = 0;
int AN_Pot1_Filtered = 0;

//int readADCVal = 0;
//-------------------------------------------------------------------------------
///////////////////////////////////BT////////////////////////////////////////////
String btText = "";
bool btConnectionStatus = false;
bool lastTime = false;


//-------------------------------------------------------------------------------
/////////////////////////////////// RTC /////////////////////////////////////////
const long RTC_OFFSET = 28800; //offset in seconds GMT+8
Ticker rtcTicker;
ESP32Time rtc(RTC_OFFSET);
//-------------------------------------------------------------------------------
/////////////////////////////////// LCD /////////////////////////////////////////
const uint16_t HA_ADDR = 0x3100;
const uint16_t ETCO2_ADDR = 0x3200;
const uint16_t RR_ADDR = 0x3300;
const uint16_t S3_ADDR = 0x3400;
const uint16_t CURVE_ADDR = 0x0310;
const uint16_t RTC_SYSVAR_ADDR = 0x0010;
const uint16_t DATA_RTN_ADDR = 0x2100;
const uint16_t RIMS_ADDR = 0x2500;
const uint16_t BATT_ADDR = 0x2800;

const int DISP_DELAY_MS = 100;
//NOTE: This lcd_buf buffer size is done in such a way to reduce the chance of LCD sending RTC or Patient name as truncated.
//As LCD does not use any \n \r, everytime ESP32 updates the graph or texts on LCD, it will send replies over.
//These replies is send together with any RTC or Patient name.
//Too big buffer will impact performance of BT connection and data output.
//Too less will impact the part where needed data cannot be received reliabily.
byte lcd_buf[84] = {};
Ticker displayTicker;
//Ticker lcdInputTicker;
bool rtcRequestFlag = false;
int rtcMaxRetries = 0;
bool initializationState = false;


//-------------------------------------------------------------------------------
///////////////////////////////// Buttons ///////////////////////////////////////
const int BTN_DEBOUNCE_MS = 50;
const int BTN_CHECK_MS = 10;
static byte buttonPin[2] = {BUTTON_1, BUTTON_2};
static byte debouncedBtnState[2] = {1, 1};
static bool buttonPressed[2] = {false, false};
static byte buttonCount[2] = {BTN_DEBOUNCE_MS/BTN_CHECK_MS, BTN_DEBOUNCE_MS/BTN_CHECK_MS};
Ticker buttonTicker;

//-------------------------------------------------------------------------------
///////////////////////////////// U600 ///////////////////////////////////////////
U600 co2Sensor(Serial2, 100, SENSOR_RXD, SENSOR_TXD);

float co2 = 0.001;
float etCo2 = 0.001;
float respirationRate = 0.001;
float inspiration = 0.001;

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
const byte MAX_NAME_LEN = 5;
String defaultLoggingFile = "/test1.csv";
char currentLoggingFile[30] = {};
File file;
boolean sdCardAvailable = false;
boolean loggingIsOn = false;
String patient = "Test1";
char name_str[MAX_NAME_LEN + 1] = {'T','e','s','t','1'}; //+1 for NULL

Ticker loggingTicker;
//----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//////////////////////////// Asthma Classification/////////////////////////////
int result0 = 0;
int result1 = 0;
int result2 = 0;
int result3 = 0;
int button0 = 0;  
int button1 = 0; 
int button2 = 0;
int button3 = 0;
int astma = 0;
//----------------------------------------------------------------------------

void resetData(){
    co2Avg = 0.00;
    etCo2 = 0.00;
    respirationRate = 0.00;
    hjorthActivity = 0.00;
    S3deg = 0.00;
}

void updateCo2Disp(){
    //commented out as graph need to consistently plot out
    //static float co2Avg_cached = 0.0;
    //if(co2Avg_cached != co2Avg){
        //co2Avg_cached = co2Avg;
        static int16_t co2AvgPrev = 0;
        int16_t co2AvgInt = co2Avg*100;
        char buf[16] = {0x5A, 0xA5, 0x0D, 0x82, HIGH_BYTE(CURVE_ADDR), LOW_BYTE(CURVE_ADDR), 0x5A, 0xA5, 0x01, 0x00, 0x00, 0x02};
        buf[12] = (co2AvgPrev >> 8) & 0xFF;
        buf[13] = (co2AvgPrev & 0xFF);
        buf[14] = (co2AvgInt >> 8) & 0xFF;
        buf[15] = (co2AvgInt & 0xFF);
        co2AvgPrev = co2AvgInt; //store current value as previous value
        Serial1.write(buf, sizeof(buf));
    //}
}

void updateEtCo2Disp(){
    //commented out as LCD requires constant refresh else value disappears
    static float etCo2_cached = -0.001;
    
    if(etCo2_cached != etCo2){
        etCo2_cached = etCo2;
        String str = String(etCo2);
        char buf[32] = {0x5A, 0xA5, 0x00, 0x82, HIGH_BYTE(ETCO2_ADDR), LOW_BYTE(ETCO2_ADDR)};

        if((str.length() + 6) <= sizeof(buf)){
            buf[2] = str.length() + 3; //update length
            memcpy(&buf[6], str.c_str(), str.length());
            Serial1.write(buf, (str.length() + 6));
        }
        else{
            Serial.println("Insufficient buffer size: ETCO2");
        }
    }
}

void updateRespirationRateDisp(){
    
    static float RR_cached = -0.001;
    if(RR_cached != respirationRate){
        RR_cached = respirationRate;

        String str = String(respirationRate);
        char buf[32] = {0x5A, 0xA5, 0x00, 0x82, HIGH_BYTE(RR_ADDR), LOW_BYTE(RR_ADDR)};

        if((str.length() + 6) <= sizeof(buf)){
            buf[2] = str.length() + 3; //update length
            memcpy(&buf[6], str.c_str(), str.length());
            Serial1.write(buf, (str.length() + 6));
        }
        else{
            Serial.println("Insufficient buffer size: Respiration Rate");
        }
    }

}

void updateHjorthActDisplay(){
    
    static float HA_cached = -0.001;
    if(HA_cached != hjorthActivity){
        HA_cached = hjorthActivity;

        String str = String(hjorthActivity);
        char buf[32] = {0x5A, 0xA5, 0x00, 0x82, HIGH_BYTE(HA_ADDR), LOW_BYTE(HA_ADDR)};

        if((str.length() + 6) <= sizeof(buf)){
            buf[2] = str.length() + 3; //update length
            memcpy(&buf[6], str.c_str(), str.length());
            Serial1.write(buf, (str.length() + 6));
        }
        else{
            Serial.println("Insufficient buffer size: Hjorth Activity");
        }
    }

}

void updateS3DegDisplay(){
    
    static float S3deg_cached = -0.001;

    if(S3deg_cached != S3deg ){
        S3deg_cached = S3deg;

        String str = String(S3deg);
        char buf[32] = {0x5A, 0xA5, 0x00, 0x82, HIGH_BYTE(S3_ADDR), LOW_BYTE(S3_ADDR)};

        if((str.length() + 6) <= sizeof(buf)){
            buf[2] = str.length() + 3; //update length
            memcpy(&buf[6], str.c_str(), str.length());
            Serial1.write(buf, (str.length() + 6));
        }
        else{
            Serial.println("Insufficient buffer size: S3(degree)");
        }
    }

}

void updateRIMSDisplay(int status){

    
    if(NORMAL == status)
    {
       //No Anomaly
       byte buf[]={0x5A,0xA5,0x0F,0x82,0x25,0x00,0x4E,0x6F,0x20,0x41,0x6E,0x6F,0x6D,0x61,0x6C,0x79,0x00,0x00};
       Serial1.write(buf,sizeof(buf));
    }
    else if(MILD == status)
    {
       //Mild
       byte buf[]={0x5A,0xA5,0x0F,0x82,0x25,0x00,0x4D,0x69,0x6C,0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
       Serial1.write(buf,sizeof(buf));
    }
    else if(SEVERE == status)
    {
       //Severe
       byte buf[]={0x5A,0xA5,0x0F,0x82,0x25,0x00,0x53,0x65,0x76,0x65,0x72,0x65,0x00,0x00,0x00,0x00,0x00,0x00};
       Serial1.write(buf,sizeof(buf));
    }
    else
    {
       //Measuring
       byte buf[]={0x5A,0xA5,0x0F,0x82,0x25,0x00,0x4D,0x65,0x61,0x73,0x75,0x72,0x69,0x6E,0x67,0x00,0x00,0x00};
       Serial1.write(buf,sizeof(buf));
    }
    
}

void updateloggingDisplay(){
     
     if(loggingIsOn){ //TEXT: logging started
          byte buf[]={0x5A,0xA5,0x13,0x82,0x35,0x00,0x4C,0x6F,0x67,0x67,0x69,0x6E,0x67,0x20,0x73,0x74,0x61,0x72,0x74,0x65,0x64,0x0A};     
          Serial1.write(buf,sizeof(buf));
     }
     else{            //TEXT: logging stopped   4C 6F 67 67 69 6E 67 20 73 74 6F 70 70 65 64 0A
          byte buf[]={0x5A,0xA5,0x13,0x82,0x35,0x00,0x4C,0x6F,0x67,0x67,0x69,0x6E,0x67,0x20,0x73,0x74,0x6F,0x70,0x70,0x65,0x64,0x0A};
          Serial1.write(buf,sizeof(buf));
     }
     
}

void updateDisplay(){
    if(initializationState){
        updateDisplayForce();
    }
}

void updateDisplayForce(){
    updateCo2Disp();
    updateEtCo2Disp();
    updateRespirationRateDisp();
    updateHjorthActDisplay();
    updateS3DegDisplay();
    updateloggingDisplay();
}

void requestCurrTime(){
    //Serial.println("start init req time");
    if(!loggingIsOn){ //only request the time if logging is not on
        char buf[7] = {0x5A, 0xA5, 0x04, 0x83, HIGH_BYTE(RTC_SYSVAR_ADDR), LOW_BYTE(RTC_SYSVAR_ADDR), 0x04};
        Serial1.flush(); //flush outgoing data if any - to ensure data reply is clean
        Serial1.write(buf, sizeof(buf));
        rtcRequestFlag = true;
        //Serial.println("request time");

    }
}

void init_ADC_Pins(){
    analogSetWidth(10);
    analogReadResolution(10);                                                    // set the ADC resolution
    pinMode(GPIO_NUM_36, INPUT);                                                // set input mode
    analogSetPinAttenuation(GPIO_NUM_36, ADC_11db);                             // ADC setting pin ADC1_CH0
}


void calc_battery_percentage(){
    int readADCVal = 0;
    int readADCValAvg = 0;
    int i = 0;
    uint32_t Sum = 0;
    readADCVal = analogRead(VBAT_ADC);
  
    AN_Pot1_Buffer[AN_Pot1_i++] = readADCVal;
    if(AN_Pot1_i == FILTER_LEN)
    {
      AN_Pot1_i = 0;
    }
    for(i=0; i<FILTER_LEN; i++)
    {
      Sum += AN_Pot1_Buffer[i];
    }
    readADCValAvg = (Sum/FILTER_LEN);
    
    int battery_percentage = 100 * (readADCValAvg - BATTERY_MIN_ADC) / (BATTERY_MAX_ADC - BATTERY_MIN_ADC);

    if (battery_percentage < 0){
        battery_percentage = 0;
    }
    else if (battery_percentage > 100){
        battery_percentage = 100;
    }
/*
    Serial.println("ADC");
    Serial.print(readADCVal);
    Serial.print("\n");
    Serial.print(battery_percentage);
    Serial.print("\n");
    Serial.print(BATTERY_MIN_ADC);
    Serial.print("\n");
    Serial.print(BATTERY_MAX_ADC);
    Serial.print("\n");
*/
    String str = String(battery_percentage) + " ";
    char buf[32] = {0x5A, 0xA5, 0x00, 0x82, HIGH_BYTE(BATT_ADDR), LOW_BYTE(BATT_ADDR)};
    
    if((str.length() + 6) <= sizeof(buf)){
         buf[2] = str.length() + 3; //update length
         memcpy(&buf[6], str.c_str(), str.length());
         Serial1.write(buf, (str.length() + 6));
     }
     else{
         Serial.println("Insufficient buffer size: BATT");
     }
    
}

void isLoggingOn(){
    static boolean current_logging_state = false;
    if(buttonPressed[0]){
        buttonPressed[0] = false; //reset the button state
        current_logging_state = !current_logging_state;
        
        if(current_logging_state){
            Serial.println("Logging started ....");
            byte second = rtc.getSecond();
            byte min = rtc.getMinute();
            byte hour = rtc.getHour(true);
            byte day = rtc.getDay();
            byte month = rtc.getMonth();
            short year = rtc.getYear();

            if (year>2000){
              year = year - 2000;
              month = month + 1;
            }

            if(hour < 8){ //GMT +8 get back the correct time
              hour = 24 + hour - 8;
              day = day - 1;
            }
            else{
              hour = hour - 8;     
            }

            char dateTimeFormat[14];
            
            //yymmdd-hhmmss-name
            sprintf(dateTimeFormat, "%02d%02d%02d-%02d%02d%02d",year,month,day,hour,min,second); 
            
            //Serial.println(dateTimeFormat);
            //Serial.println(name_str);
            snprintf(currentLoggingFile, sizeof(currentLoggingFile), "/%s-%s.csv", dateTimeFormat, name_str); 

            String header = "";
            /*
            if(patient.length()){
                header = String("TIME") + String(DELIM) +
                        String("PATIENT") + String(DELIM) +
                        String("AVGCO2") + String(DELIM) +
                        String("ETCO2") + String(DELIM) +
                        String("RR") + String(DELIM) +
                        String("AVGACTCO2") + String(DELIM) +
                        String("S3(degree)") + String(NEWLINE);
            }
            else{
                header = String("TIME") + String(DELIM) +
                        String("AVGCO2") + String(DELIM) +
                        String("ETCO2") + String(DELIM) +
                        String("RR") + String(DELIM) +
                        String("AVGACTCO2") + String(DELIM) +
                        String("S3(degree)") + String(NEWLINE);
            }*/
            header = String("TIME") + String(DELIM) +
                     String("AVGCO2") + String(DELIM) +
                     String("ETCO2") + String(DELIM) +
                     String("RR") + String(DELIM) +
                     String("AVGACTCO2") + String(DELIM) +
                     String("S3(degree)") + String(NEWLINE);
            #ifdef DEBUG_CO2
            Serial.print(header.c_str());
            #endif
            if(sdCardAvailable){
                openFile(currentLoggingFile, const_cast<char *>(header.c_str()));
            }
            
            digitalWrite(USER_LED1, HIGH);
            loggingIsOn = true;
        }
        else{
            loggingIsOn = false;
            Serial.println("Logging stopped ....");
            
            requestCurrTime(); //query RTC from LCD module
            //delay(DISP_DELAY_MS);
            //resetData();
            //updateDisplayForce();
            digitalWrite(USER_LED1, LOW);
            closeFile();
        }
    }
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

void openFile(const char * path, const char * data){
    file = SD_MMC.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(!file.print(data)){
        Serial.println("Write failed");
    }
    //file.close();
}

void closeFile(){
    file.close();
}

void writeFile(const char * path, const char * data){
    if(!file.print(data)){
          Serial.println("Append failed");
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


void checkInputCmd(){
    if(!loggingIsOn){ //check for input cmd from lcd
        if(Serial1.available() > 0){
            
            int len = Serial1.readBytesUntil(NEWLINE, lcd_buf, sizeof(lcd_buf));
            lcd_buf[len] = 0; //null terminate
            #ifdef DEBUG_LCD
            for(int i = 0; i < len; i++){
                Serial.print(lcd_buf[i], HEX);
                Serial.print("h ");
            }
            Serial.println();
            #endif

            char data_rtn_hdr[3] = {0x83, HIGH_BYTE(DATA_RTN_ADDR), LOW_BYTE(DATA_RTN_ADDR)};
            char rtc_hdr[3] = {0x83, HIGH_BYTE(RTC_SYSVAR_ADDR), LOW_BYTE(RTC_SYSVAR_ADDR)};
            char *str = NULL;

            //handle patient name - check for matching header
            if((str = strstr(reinterpret_cast<char *>(lcd_buf), data_rtn_hdr)) != NULL){
                byte name_len = *(str + 3);
                //verify the total data of at least the name len is available
                if( len > ((str - reinterpret_cast<char *>(lcd_buf)) + name_len) ){
                    byte index = (str + 4) - reinterpret_cast<char *>(lcd_buf);
                    for(int i = index; i < len; i++){
                        if(lcd_buf[i] == 0xFF){
                            byte cpy_len = (i - index) - 1;
                            if(cpy_len > MAX_NAME_LEN){
                                cpy_len = MAX_NAME_LEN;
                            }
                            memcpy(name_str, &lcd_buf[index], cpy_len);

                            //snprintf(currentLoggingFile, sizeof(currentLoggingFile), "/%s.csv", name_str);
                            patient = String(name_str);
                            #ifdef DEBUG_LCD
                            Serial.println(name_str);
                            #endif
                        }
                    }
                }    
            }
            //handle rtc reply - check for matching header
            else if((str = strstr(reinterpret_cast<char *>(lcd_buf), rtc_hdr)) != NULL){
                
                byte date_len = *(str + 3); //len in word
                if( len >= ((str - reinterpret_cast<char *>(lcd_buf)) + (date_len*2) + sizeof(rtc_hdr) + 1) ){
                    byte index = (str + 4) - reinterpret_cast<char *>(lcd_buf);
                    rtc.setTime(lcd_buf[index+6], lcd_buf[index+5], lcd_buf[index+4], lcd_buf[index+2], lcd_buf[index+1], (2000+lcd_buf[index])); //sc, mn, hr, dy, mt, yr
                    Serial.println("RTC Time Received");
                    //check whether LCD returns default value , if yes retry again. Note: Sometimes LCD will return 00 for all , thus need retry and get the correct datetime
                    if(lcd_buf[index] == 00 && rtcMaxRetries !=5){
                    
                         Serial.println("default RTC values");
                         requestCurrTime();
                         rtcRequestFlag = true;
                         rtcMaxRetries ++;
                          
                    }
                    else{
                         rtcRequestFlag = false;
                         rtcMaxRetries = 0;
                    }
                    #ifdef DEBUG_LCD
                    char time_str[9] = {};
                    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", lcd_buf[index+4], lcd_buf[index+5], lcd_buf[index+6]);
                    Serial.println(time_str);
                    #endif
                }    
            }
            
            else if (rtcRequestFlag == true && rtcMaxRetries !=5){
              Serial.println("No RTC replied");
              requestCurrTime(); 
              rtcRequestFlag = true;
              rtcMaxRetries ++;              
            }
            //handle sensor zero calibration
            //else if(){
                #ifdef U600_RECALIBRATION_TRUE
                co2Sensor.recalibration();
                #endif
            //}
        }
    }
}

void updateCo2(co2_t* co2Packet){
    co2 = co2Sensor.getCo2Concentration(co2Packet);
    z = 0.5335 * z + 0.4665 * co2;
    co2Avg = movingAverageFilter.process(z);
    if(co2Avg < 0){
         co2Avg = 0.00;       
    }
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
    double tanTheta = (m2-m1)/(1+(m1*m2));
    S3rad = atan(tanTheta);
    double tmp_S3deg = (abs(S3rad)*180)/PI;
    if(!isnan(tmp_S3deg)){
        if(tmp_S3deg < 0){
            S3deg = 180 - tmp_S3deg;
        }
        else{
            S3deg = tmp_S3deg;
        }
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
        char time_str[9] = {};
        long epoch = rtc.getLocalEpoch();
        byte sec = epoch % 60; 
        epoch /= 60;
        byte min = epoch % 60; 
        epoch /= 60;
        byte hour = epoch % 24; 
        epoch /= 24;
        snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", hour, min, sec);
        


        String log = "";
        String co2_data = String(co2Avg) + String(DELIM) +
                        String(etCo2) + String(DELIM) +      
                        String(respirationRate) + String(DELIM) +
                        String(hjorthActivity) + String(DELIM) +
                        String(S3deg) + String(NEWLINE);
        //prepend patient name if available
        //if(patient.length()){
            //log = String(time_str) + String(DELIM) + patient + String(DELIM) + co2_data;
        //}
        //else{
            log = String(time_str) + String(DELIM) + co2_data;
        //}
        #ifdef DEBUG_CO2
        Serial.print(log.c_str());
        #endif
        if(sdCardAvailable){
            writeFile(currentLoggingFile, const_cast<char *>(log.c_str()));
        }
    }
}

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
    if(event == ESP_SPP_SRV_OPEN_EVT){
       Serial.println("BT connected");
       btConnectionStatus = true;
       //BT connected         42 54 20 63 6F 6E 6E 65 63 74 65 64
       byte buf[]={0x5A,0xA5,0x12,0x82,0x36,0x00,0x42,0x54,0x20,0x63,0x6F,0x6E,0x6E,0x65,0x63,0x74,0x65,0x64,0x00,0x00,0x00};     
       Serial1.write(buf,sizeof(buf));
       
    }

    else if(event == ESP_SPP_CLOSE_EVT){
       Serial.println("BT disconnected");
       btConnectionStatus = false;
       lastTime = 0;
       //BT disconnected 42 54 20 64 69 73 63 6F 6E 6E 65 63 74 65 64
       byte buf[]={0x5A,0xA5,0x12,0x82,0x36,0x00,0x42,0x54,0x20,0x64,0x69,0x73,0x63,0x6F,0x6E,0x6E,0x65,0x63,0x74,0x65,0x64};
       Serial1.write(buf,sizeof(buf));
    }
}

void setup() {
    Serial.begin(115200);
    Serial.print("I-Breath CO2 Device ");
    Serial.println(app_ver);
    Serial.println("System initialization ...");

    xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "serialTask",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &serialTask,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */
    delay(500);
    
    for(int i=0; i< MAX_CAPNO_LENGTH; i++){
        timeAxis[i] = i;
    }

    //LCD serial line
    Serial1.begin(115200, SERIAL_8N1, LCD_RX2, LCD_TX2);



    Serial.println("... Initializing CO2 sensor.");
    co2Sensor.init();

    SerialBT.register_callback(callback);
    //Bluetooth device name
    if(!SerialBT.begin("I-Breath3")){
        Serial.println("... BT initialization error"); 
    }
    else{
        Serial.println("... BT initialized");
    }
      
    memcpy(currentLoggingFile, defaultLoggingFile.c_str(), defaultLoggingFile.length());

    #ifdef SDCARD_INIT
    Serial.println("... Initializing SD card.");
    if(SD_MMC.begin()){
        if(SD_MMC.cardType() != CARD_NONE)
        {
            Serial.println("... SD card is available.");
            sdCardAvailable = true;
        }
    }
    #endif

    //overwrite LCD time
    //Serial1.write(TIME, sizeof(TIME));

    //init adc pins
    init_ADC_Pins();

    Serial.println("... Initializing I/O.");
    pinMode(USER_LED1, OUTPUT);
    pinMode(buttonPin[0], INPUT);
    pinMode(buttonPin[1], INPUT);
    buttonTicker.attach_ms(BTN_CHECK_MS, debounceBtnSWRoutine, 0); //debouncing for button[0]
    displayTicker.attach_ms(DISP_DELAY_MS, updateDisplay);
    //lcdInputTicker.attach_ms(DISP_DELAY_MS, checkInputCmd);
    
    Serial.println("... Initialization completed. Push button to start logging.");
    initializationState = true;
    requestCurrTime(); //query RTC from LCD module
    
    
    
}
//core 0 task
void Task1code( void * pvParameters ){

  for(;;){
    
    if(initializationState){
       checkInputCmd();
       calc_battery_percentage();     
    }
    vTaskDelay(1);
  }

}

void updateAsthmaSeverity(){
  //Turns Count algorithm for classification of asthma severity
  if ((etCo2 >= 32) && (etCo2 <= 45) && (respirationRate > 10) && (respirationRate < 30)){
    result1 = result1 + 1;
    result0 = 0;
    result2 = 0;
    result3 = 0;
  }

  if ((button1 == 1) && (etCo2 == 0) && (respirationRate == 0)){
    button1 = 0;
  }

  if ((result1 == 20) && (button1 == 0)){
    updateRIMSDisplay(NORMAL); 
    astma = 1;
    result1 = 0;
    button1 = 1;
  }


  if ((etCo2 < 32) && (respirationRate > 28)){
    result2 = result2 + 1;
    result0 = 0;
    result1 = 0;
    result3 = 0;
  }

  if ((button2 == 1) && (etCo2 == 0) && (respirationRate == 0)){
    button2 = 0;
  }

  if ((result2 == 25) && (button2 == 0)){
    updateRIMSDisplay(MILD); 
    astma = 5;
    result2 = 0;
    button2 = 1;
  }
  
  if ((etCo2 >= 45 ) && (respirationRate < 8)){
    result0 = result0 + 1;
    result1 = 0;
    result2 = 0;
    result3 = 0;
  }

  if ((button0 == 1) && (etCo2 == 0) && (respirationRate == 0)){
    button0 = 0;
  }

  if ((result0 == 8) && (button0 == 0)){
    updateRIMSDisplay(SEVERE);
    astma = 10;
    result0 = 0;
    button0 = 1;
  }

  if ((etCo2 <= 5 ) ||  (respirationRate <= 5)){
    result3 = result3 + 1;
    result0 = 0;
    result1 = 0;
    result2 = 0;
  }

  if ((button3 == 1) && (etCo2 == 0) && (respirationRate == 0)){
    button3 = 0;
  }

  if ((result3 == 4) && (button3 == 0)){
    updateRIMSDisplay(MEASURING);
    astma = 0;
    result3 = 0;
    button3 = 1;
  }
}

void sendBTData() {

  if (btConnectionStatus) {
     if(lastTime == true){
        lastTime=false; 
        btText = "#" + String(co2Avg) + "," + String(patient) + "\n";
        SerialBT.print(btText.c_str());     
     }
     else{
        lastTime=true;
     }
  }

}

void loop() {
    
    isLoggingOn();  //sdcard file setup
    co2Sensor.read();
    while(co2Sensor.isAvailable()){
        co2_t* co2Packet = co2Sensor.getCo2Reading();
        if(co2Packet!=NULL){
            updateCo2(co2Packet);
            updateEtCo2(co2Packet);
            updateRespirationRate(co2Packet);
            updateInspiration(co2Packet);
            doLogging(); //logging to file
            updateAsthmaSeverity();
            sendBTData();
            
        }
    }
}
