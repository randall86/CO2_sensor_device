#include "Arduino.h"
#include <HardwareSerial.h>
#include "u600.h"

#define RXD2 16
#define TXD2 17

U600::U600(HardwareSerial& serial, uint16_t readBufferSize):_Serial2(serial){
  rxReadBuffer = new uint8_t[readBufferSize];  
  duplicate  = -1;

  packetAvailable = 0;
  rxInIndex = 0;
  rxOutIndex = 0;
  rxCurrentPacket = &packetBuffer[rxInIndex];
  rxCurrentBuffer = (uint8_t*)(rxCurrentPacket);
}

void U600::init(){
  rxState = CMD;
  _Serial2.begin(19200,SERIAL_8N1, RXD2, TXD2); //U600 Sensor (PB10/TX3 PB11/RX3)
}

void U600::read(){
  uint8_t rxBytes = _Serial2.available(); //Internal RX buffer (64 bytes)
  if(rxBytes > 0){
     _Serial2.readBytes(rxReadBuffer,rxBytes); //Flush internal buffer to READ buffer
     for(int i=0; i<rxBytes; i++){
        packetParser(rxReadBuffer[i]);
     }
  }
}

int U600::isAvailable(){
  return packetAvailable;
}

co2_t* U600::getCo2Reading(){
  if(rxInIndex == rxOutIndex){
    return NULL;
  }
  uint8_t* co2out = (uint8_t*)&packetBuffer[rxOutIndex];
  uint8_t* outputPtr = (uint8_t*)&output;
  for(int i=0; i<sizeof(co2_t); i++){
    outputPtr[i] = co2out[i];
  }
  packetAvailable = packetAvailable - 1;
  rxOutIndex = (rxOutIndex+1)%PACKET_BUFFER_SIZE;
//  printBufferToSerial(outputPtr);
  return &output;
} 

boolean U600::isInfo(co2_t* _packet){
  if(_packet->dpi == 1){
    return true;
  }
  return false;
}
boolean U600::isEtCo2(co2_t* _packet){
  if(_packet->dpi == 2){
    return true;
  }
  return false;
}
boolean U600::isRespirationRate(co2_t* _packet){
  if(_packet->dpi == 3){
    return true;
  }
  return false;
}
boolean U600::isInspiration(co2_t* _packet){
  if(_packet->dpi == 4){
    return true;
  }
  return false;
}
boolean U600::isBreathComplete(co2_t* _packet){
  if(_packet->dpi == 5){
    return true;
  }
  return false;
}
boolean U600::isHardwareStatus(co2_t* _packet){
  if(_packet->dpi == 7){
    return true;
  }
  return false;
}

float U600::getCo2Concentration(co2_t* _packet){
  return ((128*_packet->co2MSB + _packet->co2LSB) - 1000) / 100.0;
}
float U600::getEtCo2(co2_t* _packet){
  return ((_packet->dpiStatus[0]*128) + _packet->dpiStatus[1])/10.0;
}
float U600::getRespirationRate(co2_t* _packet){
  return (_packet->dpiStatus[0]*128) + _packet->dpiStatus[1];
}
float U600::getInspiration(co2_t* _packet){
  return (_packet->dpiStatus[0]*128 + _packet->dpiStatus[1])/10.0;
}

void U600::packetParser(uint8_t rxByte){
   switch(rxState){
      case CMD:
        if(rxByte == 0x80){
          clearRxBuffer();
          rxCount = 0;
          rxCurrentBuffer[rxCount++] = rxByte;
          rxState = NBF;
        }
      break;
      case NBF:
        rxCurrentBuffer[rxCount++] = rxByte;
        nCount = 0;
        rxState = DBN;
      break;
      case DBN:
        if(nCount < rxCurrentPacket->nbf-1){
          rxCurrentBuffer[rxCount++] = rxByte;
        }else{
          rxCurrentPacket->chs = rxByte;
          packetVerification();
          rxState = CMD;
        }
        nCount++;
      break;
   }
}

void U600::packetVerification(){
  uint16_t sum = 0;
  for (int i=0; i<rxCurrentPacket->nbf+1;i++){
    sum +=  rxCurrentBuffer[i];
  }
  sum = ((~sum)+1) & 0x7F;
  
  if(sum ==  rxCurrentPacket->chs){
    if(rxCurrentPacket->seq == duplicate){
      return;
    }
    duplicate = rxCurrentPacket->seq;
    #ifdef _U600_VERBOSE_
      printBufferToSerial(rxCurrentBuffer);
      printToSerial(rxCurrentBuffer);
    #endif

    if(((rxInIndex+1)%PACKET_BUFFER_SIZE) == rxOutIndex){
      return;
    }
    packetAvailable = packetAvailable+1;
    rxInIndex = (rxInIndex+1)%PACKET_BUFFER_SIZE;
    rxCurrentPacket = &packetBuffer[rxInIndex];
    rxCurrentBuffer = (uint8_t*)rxCurrentPacket;
  } 
}

void U600::clearRxBuffer(){
  for(int i=0; i<sizeof(co2_t); i++){
    rxCurrentBuffer[i] = 0;
  }
}
