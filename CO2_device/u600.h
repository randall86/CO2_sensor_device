#ifndef __U600__
#define __U600__

#include "Arduino.h"
#include <HardwareSerial.h>

typedef enum _balanceGas {
    ROOM_AIR,
    N20,
    HELIUM,
    INVALID
}balanceGas_t;

typedef enum _tempStatus {
    TEMP_NORMAL,
    TOO_LOW,
    TOO_HIGH,
    TEMP_UNSTABLE
}temp_t;

typedef enum _calibrationStatus {
    CAL_NORMAL,
    CAL_IN_PROG,
    CAL_ANOMALLY, //tube disconnected/occluding/CO2 -ve detected
    CAL_FAILED
}calib_t;

typedef struct {
  uint8_t cmd;
  uint8_t nbf;
  uint8_t seq;
  uint8_t co2MSB;
  uint8_t co2LSB;
  uint8_t dpi;
  uint8_t dpiStatus[5];
  uint8_t chs;
}co2_t;

typedef struct {
  //DB1
  uint8_t CO2_is_negative;
  uint8_t nose_tube_disconnected;
  uint8_t last_calibration;
  uint8_t CO2_out_of_range; //(150 mmHg, 20.0 kPa or 19.7 %)
  uint8_t ready_for_calibration;
  uint8_t no_breath_detected;
  //DB2
  uint8_t atmosphere_pressure_anesthetic_set;
  //DB5
  uint8_t system_info;
}co2_info_t;

typedef struct {
  uint8_t nose_tube_disconnected;
  uint8_t pump_exceeds_life;
  uint8_t nose_tube_occluded;
  uint8_t pump_is_off;
}co2_pump_t;

class U600{
  public:
  
    U600(HardwareSerial& serial, uint16_t readBufferSize, int8_t rxPin = -1, int8_t txPin = -1);
    void init(uint16_t atmosphere = 760, uint8_t o2 = 16, balanceGas_t bal = ROOM_AIR, float anes = 0.0);
    void read();
    int isAvailable();
    co2_t* getCo2Reading();
    void recalibration();
    void resetSystem();

    boolean isInfo(co2_t* _packet);
    boolean isEtCo2(co2_t* _packet);
    boolean isRespirationRate(co2_t* _packet);
    boolean isInspiration(co2_t* _packet);
    boolean isBreathComplete(co2_t* _packet);
    boolean isHardwareStatus(co2_t* _packet);
    boolean isHardwareStatusError(co2_t* _packet);

    float getEtCo2(co2_t* _packet);
    float getRespirationRate(co2_t* _packet);
    float getCo2Concentration(co2_t* _packet);
    float getInspiration(co2_t* _packet);
    void getCo2SysInfo(co2_t* _packet, co2_info_t &info);
    temp_t getSysTemp(co2_t* _packet);
    calib_t getCalibrationState(co2_t* _packet);
    void getCo2PumpInfo(co2_t* _packet, co2_pump_t &info);
    
    void setAtmospherePressure(uint16_t atmosphere);
    void setAnestheticGasCompensation(uint8_t o2, balanceGas_t bal, float anes);
    
  private:
    HardwareSerial& _Serial;
    int8_t _rxPin;
    int8_t _txPin;
    static const uint8_t PACKET_BUFFER_SIZE = 500;
    enum RX_STATE{   
      CMD = 0, 
      NBF = 1, 
      DBN = 2
    } rxState;
    uint8_t* rxReadBuffer;
    
    co2_t packetBuffer[PACKET_BUFFER_SIZE];
    co2_t*  rxCurrentPacket;
    uint8_t* rxCurrentBuffer;
    uint16_t rxInIndex;
    uint16_t rxOutIndex;

    co2_t output;
    int packetAvailable;
    
    int rxCount, nCount, duplicate;
    
    void clearRxBuffer();
    
    void packetParser(uint8_t rxByte);
    void packetVerification();
    
    void checkUplink();
};

#endif
