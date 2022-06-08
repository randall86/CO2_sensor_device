#include "Arduino.h"
#include <HardwareSerial.h>
#include "u600.h"

//#define _U600_VERBOSE_

#ifdef _U600_VERBOSE_
void printBufferToSerial(const char * string, uint8_t* buffer, uint16_t bufferSize){
  Serial.print(string);
  for(int i = 0; i < bufferSize; i++)
  {
    Serial.print(buffer[i], HEX);
    Serial.print("h ");
  }
  Serial.println();
}
#endif

U600::U600(HardwareSerial& serial, uint16_t readBufferSize, int8_t rxPin, int8_t txPin): 
  _rxPin(rxPin),
  _txPin(txPin),
  _Serial(serial)
{
  rxReadBuffer = new uint8_t[readBufferSize];  
  duplicate  = -1;

  packetAvailable = 0;
  rxInIndex = 0;
  rxOutIndex = 0;
  rxCurrentPacket = &packetBuffer[rxInIndex];
  rxCurrentBuffer = (uint8_t*)(rxCurrentPacket);
}

void U600::init(uint16_t atmosphere, uint8_t o2, balanceGas_t bal, float anes){
  rxState = CMD;
#ifdef ARDUINO_ARCH_ESP32
  _Serial.begin(19200, SERIAL_8N1, _rxPin, _txPin); //U600 Sensor (PB10/TX3 PB11/RX3)
#else
  _Serial.begin(19200, SERIAL_8N1); //U600 Sensor (PB10/TX3 PB11/RX3)
#endif
  checkUplink();
  setAtmospherePressure(atmosphere);
  setAnestheticGasCompensation(o2, bal, anes);
}

void U600::read(){
  uint8_t rxBytes = _Serial.available(); //Internal RX buffer (64 bytes)
  if(rxBytes > 0){
     _Serial.readBytes(rxReadBuffer,rxBytes); //Flush internal buffer to READ buffer
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
  #ifdef _U600_VERBOSE_
    printBufferToSerial("Extract: ", outputPtr, sizeof(co2_t));
  #endif
  return &output;
}

void U600::recalibration(){
  if(_Serial.availableForWrite()){
    //Calibration CO2 Sensor
    _Serial.write(0x82);
    _Serial.write(0x00);
    _Serial.write(0x7E); //CKS = (~(0x82 + 0x00)+1) & 0x7F
    delay(1000);
  }
}

void U600::resetSystem(){
  if(_Serial.availableForWrite()){
    //Reset CO2 Sensor
    _Serial.write(0xF8);
    _Serial.write(0x01);
    _Serial.write(0x07); //CKS = (~(0xF8 + 0x01)+1) & 0x7F
    delay(1000);
  }
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
boolean U600::isHardwareStatusError(co2_t* _packet){
  if(_packet->dpi == 7){
    if((_packet->dpiStatus[0] != 0) && (_packet->dpiStatus[1] != 0)){
      return true;
    }
    return false;
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
void U600::getCo2SysInfo(co2_t* _packet, co2_info_t &info){
  //DB1
  info.CO2_is_negative = (_packet->dpiStatus[0] & 0x01);
  info.nose_tube_disconnected = (_packet->dpiStatus[0] & 0x02);
  info.last_calibration = (_packet->dpiStatus[0] & 0x04);
  info.CO2_out_of_range = (_packet->dpiStatus[0] & 0x08);
  info.ready_for_calibration = (_packet->dpiStatus[0] & 0x10);
  info.no_breath_detected = (_packet->dpiStatus[0] & 0x40);
  //DB2
  info.atmosphere_pressure_anesthetic_set = (_packet->dpiStatus[1] & 0x10);
  //DB5
  info.system_info = _packet->dpiStatus[4];
}
temp_t U600::getSysTemp(co2_t* _packet){
    return (temp_t)(_packet->dpiStatus[1] & 0x03);
}
calib_t U600::getCalibrationState(co2_t* _packet){
    return (calib_t)(_packet->dpiStatus[1] & 0x0C);
}
void U600::getCo2PumpInfo(co2_t* _packet, co2_pump_t &info){
  info.nose_tube_disconnected = (_packet->dpiStatus[3] & 0x01);
  info.pump_exceeds_life = (_packet->dpiStatus[3] & 0x02);
  info.nose_tube_occluded = (_packet->dpiStatus[3] & 0x04);
  info.pump_is_off = (_packet->dpiStatus[3] & 0x08);
}

void U600::setAtmospherePressure(uint16_t atmosphere){
  if(_Serial.availableForWrite()){
    uint8_t db1 = (uint8_t)(atmosphere/128);
    uint8_t db2 = (uint8_t)(atmosphere%128);
    uint8_t cks = ((~(0x89 + db1 + db2))+1) & 0x7F;
    uint8_t buf[6] = {0x84, 0x04, 0x01, db1, db2, cks};
    _Serial.write(buf, sizeof(buf));
  }
}

void U600::setAnestheticGasCompensation(uint8_t o2, balanceGas_t bal, float anes){
  if(_Serial.availableForWrite()){
    uint8_t db1 = o2;
    uint8_t db2 = (uint8_t)bal;
    uint8_t db3 = ((uint8_t)anes*10)/128;
    uint8_t db4 = ((uint8_t)anes*10)%128;;
    uint8_t cks = ((~(0x95 + db1 + db2 + db3 + db4))+1) & 0x7F;
    uint8_t buf[8] = {0x84, 0x06, 0x0B, db1, db2, db3, db4, cks};
    _Serial.write(buf, sizeof(buf));
  }
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
      printBufferToSerial("Read: ", rxCurrentBuffer, sizeof(co2_t));
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

void U600::checkUplink(){
  uint8_t buf[4] = {0x80, 0x02, 0x00, 0x7E};
  uint8_t ret = 0;
  do{
      _Serial.write(buf, sizeof(buf));
      if (_Serial.available() > 0) {
        ret = _Serial.read();
      }
  }while(ret != 0x80);
  
}
