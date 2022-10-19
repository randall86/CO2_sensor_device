# CO2_sensor_device
A CO2 sensor medical device utilizing Espressif's ESP32-WROOM-32E


DWIN_SET is the folder/files to program the LCD directly

lcd folder is the source code for the LCD GUI. To be used by DGUS programme

For ESP32 software, both CO2_device and libraries are needed

Bluetooth device name should be changed according to the device using this code in setup "SerialBT.begin("I-Breath3");"

To update LCD time , uncomment the lines below and update the code below to the current time



//Temporary LCD write time  20 Sept 2022 , 03:45:00 AM  0x16,0x09,0x14,0x03,0x2D,0x00 represents YY,MM,DD,HH,Min,SS
//byte TIME[]={0x5A,0xA5,0x0B,0x82,0x00,0x9C,0x5A,0xA5,0x16,0x09,0x14,0x03,0x2D,0x00};


void setup(){

    //overwrite LCD time
    //Serial1.write(TIME, sizeof(TIME));
