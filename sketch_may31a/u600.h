#ifndef __U600__
#define __U600__

#include "Arduino.h"
#include <HardwareSerial.h>

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
    
class U600{
  public:
  
    U600(HardwareSerial& serial,uint16_t readBufferSize);
    void init();
    void read();
    int isAvailable();
    co2_t* getCo2Reading();

    boolean isInfo(co2_t* _packet);
    boolean isEtCo2(co2_t* _packet);
    boolean isRespirationRate(co2_t* _packet);
    boolean isInspiration(co2_t* _packet);
    boolean isBreathComplete(co2_t* _packet);
    boolean isHardwareStatus(co2_t* _packet);

    float getEtCo2(co2_t* _packet);
    float getRespirationRate(co2_t* _packet);
    float getCo2Concentration(co2_t* _packet);
    float getInspiration(co2_t* _packet);
     
  private:
    HardwareSerial& _Serial2;
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
};

#endif
