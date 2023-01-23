#include <Wire.h>
#include <HoneywellTruStabilitySPI.h>
#include "SPI.h"

#define FLOW_SENSOR_ADDRESS             0x40
#define FLOW_ID_REGISTER_HIGH           0x77
#define FLOW_ID_REGISTER_LOW            0x00
#define FLOW_TEMP_REGISTER_HIGH         0x10
#define FLOW_TEMP_REGISTER_LOW          0x01
#define FLOW_TEMP_SCALE_REGISTER_HIGH   0x31
#define FLOW_TEMP_SCALE_REGISTER_LOW    0xAC
#define FLOW_TEMP_OFFSET_REGISTER_HIGH  0x31
#define FLOW_TEMP_OFFSET_REGISTER_LOW   0xAD
#define FLOW_SCALE_REGISTER_HIGH        0x30
#define FLOW_SCALE_REGISTER_LOW         0xDE
#define FLOW_OFFSET_REGISTER_HIGH       0x30
#define FLOW_OFFSET_REGISTER_LOW        0xDF
#define FLOW_REGISTER_HIGH              0x10
#define FLOW_REGISTER_LOW               0x00



#define flowResetPin 7

#define SLAVE_SELECT_PIN 10
TruStabilityPressureSensor sensor( SLAVE_SELECT_PIN, 0, 5 );


int flowSensorID;
int flowSensorTemp;
float flowSensorTempC;
int flowSensorTempScale,flowSensorTempOffset;
int flowSensorScale,flowSensorOffset;
int flowSensorVal;
float flowSensorValSLM;
int idChecksum;
float prevTime;
float flowPressure=0;
float angle=0;

void setup() {
  
  pinMode(flowResetPin,OUTPUT);
  digitalWrite(flowResetPin,LOW);
  delay(100);
  digitalWrite(flowResetPin,HIGH);
  
  Wire.begin();
  SPI.begin();
  Serial.begin(115200);
  flowSensorInit();
  sensor.begin();
  prevTime = micros();
  
}
void sendToPC(int* data1, int* data2, int* data3,int data4)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte* byteData4 = (byte*)(data4);
  byte buf[8] = {byteData1[0], byteData1[1],
                 byteData2[0], byteData2[1],
                 byteData3[0], byteData3[1],
                 byteData4[0], byteData4[1]};
  Serial.write(buf, 8);
}
void sendToPC(float* data1, float* data2, float* data3,float* data4)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte* byteData4 = (byte*)(data4);
  byte buf[16] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                 byteData4[0], byteData4[1], byteData4[2], byteData4[3]};
  Serial.write(buf, 16);
}

void flowSensorInit(void){
  //read ID register to check sensor is online
  Wire.beginTransmission(FLOW_SENSOR_ADDRESS);
  Wire.write(byte(FLOW_ID_REGISTER_HIGH));
  Wire.write(byte(FLOW_ID_REGISTER_LOW));
  Wire.endTransmission();
  delay(1);

  Wire.requestFrom(FLOW_SENSOR_ADDRESS,3);

  if (3<= Wire.available()){
    flowSensorID=Wire.read();
    flowSensorID = flowSensorID << 8;
    flowSensorID |= Wire.read();
    idChecksum = Wire.read(); 
  }
/*
  Serial.print("flow sensor ID is: ");
  Serial.println(flowSensorID, HEX);
  //Serial.print("ID checksum is: ");
  //Serial.println(idChecksum,HEX);
 */
  
  //read temp scale register for temp calibration
  Wire.beginTransmission(FLOW_SENSOR_ADDRESS);
  Wire.write(byte(FLOW_TEMP_SCALE_REGISTER_HIGH));
  Wire.write(byte(FLOW_TEMP_SCALE_REGISTER_LOW));
  Wire.endTransmission();
  delay(1);

  Wire.requestFrom(FLOW_SENSOR_ADDRESS,3);

  if (3<= Wire.available()){
    flowSensorTempScale=Wire.read();
    flowSensorTempScale = flowSensorTempScale << 8;
    flowSensorTempScale |= Wire.read();
    idChecksum = Wire.read(); 
  }
/*
  Serial.print("flow sensor temp scaling is: ");
  Serial.println(flowSensorTempScale, DEC);
*/
  //read temp offset register for temp calibration
  Wire.beginTransmission(FLOW_SENSOR_ADDRESS);
  Wire.write(byte(FLOW_TEMP_OFFSET_REGISTER_HIGH));
  Wire.write(byte(FLOW_TEMP_OFFSET_REGISTER_LOW));
  Wire.endTransmission();
  delay(1);

  Wire.requestFrom(FLOW_SENSOR_ADDRESS,3);

  if (3<= Wire.available()){
    flowSensorTempOffset=Wire.read();
    flowSensorTempOffset = flowSensorTempOffset << 8;
    flowSensorTempOffset |= Wire.read();
    idChecksum = Wire.read(); 
  }
/*
  Serial.print("flow sensor temp offset is: ");
  Serial.println(flowSensorTempOffset, DEC);
  */

  int flowTemp=getFlowSensTemperature();
  /*
  Serial.print("flow sensor temp (C) is: ");
  Serial.println(flowTemp, DEC);
*/
  //read flow scale register for temp calibration
  Wire.beginTransmission(FLOW_SENSOR_ADDRESS);
  Wire.write(byte(FLOW_SCALE_REGISTER_HIGH));
  Wire.write(byte(FLOW_SCALE_REGISTER_LOW));
  Wire.endTransmission();
  delay(1);

  Wire.requestFrom(FLOW_SENSOR_ADDRESS,3);

  if (3<= Wire.available()){
    flowSensorScale=Wire.read();
    flowSensorScale = flowSensorScale << 8;
    flowSensorScale |= Wire.read();
    idChecksum = Wire.read(); 
  }
/*
  Serial.print("flow sensor scaling is: ");
  Serial.println(flowSensorScale, DEC);
*/
  //read flow offset register for temp calibration
  Wire.beginTransmission(FLOW_SENSOR_ADDRESS);
  Wire.write(byte(FLOW_OFFSET_REGISTER_HIGH));
  Wire.write(byte(FLOW_OFFSET_REGISTER_LOW));
  Wire.endTransmission();
  delay(1);

  Wire.requestFrom(FLOW_SENSOR_ADDRESS,3);

  if (3<= Wire.available()){
    flowSensorOffset=Wire.read();
    flowSensorOffset = flowSensorOffset << 8;
    flowSensorOffset |= Wire.read();
    idChecksum = Wire.read(); 
  }
/*
  Serial.print("flow sensor offset is: ");
  Serial.println(flowSensorOffset, DEC);
  
  
  Serial.println("Done initialization");
  

  delay(3000);
 */
   
}

float getFlowSensTemperature(){
  //read ID register to check sensor is online
  Wire.beginTransmission(FLOW_SENSOR_ADDRESS);
  Wire.write(byte(FLOW_TEMP_REGISTER_HIGH));
  Wire.write(byte(FLOW_TEMP_REGISTER_LOW));
  Wire.endTransmission();
  delay(1);

  Wire.requestFrom(FLOW_SENSOR_ADDRESS,3);

  if (3<= Wire.available()){
    flowSensorTemp=Wire.read();
    flowSensorTemp = flowSensorTemp << 8;
    flowSensorTemp |= Wire.read();
    idChecksum = Wire.read(); 
  }

  flowSensorTempC = (flowSensorTemp - flowSensorTempOffset)/((float)(flowSensorTempScale));
  return flowSensorTempC; 

}

float getFlowSensValue(){
  //read ID register to check sensor is online
  Wire.beginTransmission(FLOW_SENSOR_ADDRESS);
  Wire.write(byte(FLOW_REGISTER_HIGH));
  Wire.write(byte(FLOW_REGISTER_LOW));
  Wire.endTransmission();
  delay(1);

  Wire.requestFrom(FLOW_SENSOR_ADDRESS,3);

  if (3<= Wire.available()){
    flowSensorVal=Wire.read();
    flowSensorVal = flowSensorVal << 8;
    flowSensorVal |= Wire.read();
    idChecksum = Wire.read(); 
    
  }

  flowSensorValSLM = (flowSensorVal - flowSensorOffset)/((float)(flowSensorScale));
  return flowSensorValSLM; 

}

void loop() {
 
  float curTime=micros();
  //float delta=(curTime - prevTime)*0.001;

  //real data  
  float flow=getFlowSensValue();
  float flowTemp=getFlowSensTemperature();
  if(sensor.readSensor()==0){
    flowPressure = sensor.pressure();
  }

/*
  Serial.print(curTime);
  Serial.print(',');
  Serial.print(flow);
  Serial.print(',');
  Serial.print(flowTemp);
  Serial.print(',');
  Serial.println(flowPressure);
  delay(100);
*/
/*
  //test data for python integration
  if (angle >= 360){
    angle=0;
  }
  else{
    angle += 2;
  }

  float angleRads=angle*3.1417/180;
  float flowSimulation= 1.5+sin(angleRads);
  float flowTemp= 2.5;
  float pressSimulation= 2.5+0.5*sin(angleRads);
 
  Serial.print(timeStamp);
  Serial.print(',');
  Serial.print(flowSimulation);
  Serial.print(',');
  Serial.print(random(20,24));
  Serial.print(',');

  Serial.println(pressSimulation);
*/

  //*
  sendToPC(&curTime,&flow,&flowTemp,&flowPressure);
  //prevTime=curTime;
  //Serial.println(delta);
  delay(3);
//*/
  
  
}
