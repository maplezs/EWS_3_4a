#define MAX_EASING_SERVOS 1
#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <ArduinoJson.h>
#include "ServoEasing.hpp"
#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 51; 
const int DXL_ID_ONE = 7; 
const int DXL_ID_TWO = 2; 
const int DXL_ID_THREE = 1; 
const float DXL_PROTOCOL_VERSION = 1.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;
ServoEasing gripper;
float gain1[6];
float gain2[6];
float gain3[6];
JsonDocument doc;
struct movingData {
  float theta[3];
  float prevTheta[3];
  float current[3];
  float target[3];
  int loopLen;
};
uint16_t torque1;
uint16_t torque2;
uint16_t torque3;
movingData data;
const unsigned long gripperEventInterval = 1000;
const unsigned long setupEventInterval = 1000;
const unsigned long dataEventInterval = 100;
const unsigned long moveEventInterval = 200;
unsigned long previousTime = 0;
unsigned long currentTime = 0;
void movingStatusCheck() {
  int movingStatus = 1;
  uint8_t moving_returned_one;
  uint8_t moving_returned_two;
  uint8_t moving_returned_three;
  while (movingStatus) {
    dxl.read(DXL_ID_ONE, 46, 1, (uint8_t*)&moving_returned_one, sizeof(moving_returned_one), 10);
    dxl.read(DXL_ID_TWO, 46, 1, (uint8_t*)&moving_returned_two, sizeof(moving_returned_two), 10);
    dxl.read(DXL_ID_THREE, 46, 1, (uint8_t*)&moving_returned_three, sizeof(moving_returned_three), 10);
    if (!moving_returned_one && !moving_returned_two && !moving_returned_three) {
      currentTime = millis();
      while(currentTime - previousTime < moveEventInterval){
        currentTime = millis();
      }
      previousTime = currentTime;
      movingStatus = 0;
    }
  }
}

float* calcForwardKinematic() {
  static float resultFK[3];
  float curr1 = data.current[0] * DEG_TO_RAD;
  float curr2 = data.current[1] * DEG_TO_RAD;
  float curr3 = data.current[2] * DEG_TO_RAD;
  float l1 = 0.133;
  float l2 = 0.15;
  float l3 = 0.18;
  float x = (l3*cos(curr2-curr3)+l2*cos(curr2))*cos(curr1);
  float y = (l3*cos(curr2-curr3)+l2*cos(curr2))*sin(curr1);
  float z = l3*sin(curr2-curr3)+l2*sin(curr2)+l1;
  resultFK[0] = abs(x);
  resultFK[1] = y;
  resultFK[2] = abs(z);
  
  return resultFK;
}

float* calcTorque() {
  static float result[9];
  float curr1 = data.current[0] * DEG_TO_RAD;
  float curr2 = data.current[1] * DEG_TO_RAD;
  float curr3 = data.current[2] * DEG_TO_RAD;

  float torsi1 = (-gain1[0] * data.target[0]) + (-gain1[1] * data.target[1]) + (-gain1[2] * data.target[2]) + (-gain1[3] * (data.target[0] - curr1)) + (-gain1[4] * (data.target[1] - curr2)) + (-gain1[5] * (data.target[2] - curr3));
  float torsi2 = (-gain2[0] * data.target[0]) + (-gain2[1] * data.target[1]) + (-gain2[2] * data.target[2]) + (-gain2[3] * (data.target[0] - curr1)) + (-gain2[4] * (data.target[1] - curr2)) + (-gain2[5] * (data.target[2] - curr3));
  float torsi3 = (-gain3[0] * data.target[0]) + (-gain3[1] * data.target[1]) + (-gain3[2] * data.target[2]) + (-gain3[3] * (data.target[0] - curr1)) + (-gain3[4] * (data.target[1] - curr2)) + (-gain3[5] * (data.target[2] - curr3));
  
  result[0] = ceil(abs(torsi1 * (1023 / 1.5)));
  result[1] = abs(torsi1);
  result[2] = ceil(abs(torsi2 * (1023 / 1.5)));
  result[3] = abs(torsi2);
  result[4] = ceil(abs(torsi3 * (1023 / 1.5)));
  result[5] = abs(torsi3);
  result[6] = ceil(abs(torsi1 * (100 / 1.5)));
  result[7] = ceil(abs(torsi2 * (100 / 1.5)));
  result[8] = ceil(abs(torsi3 * (100 / 1.5)));

// satu
  if (result[0] > 1023){
    result[0] = 1023;
  } else if (result[0] < 100){
    result[0] = 100;
  }
  if (result[1] > 1.5){
    result[1] = 1.5;
  }
  if (result[6] > 100){
    result[6] = 100;
  } else if (result[6] < 1){
    result[6] = 1;
  }

// dua 
  if (result[2] > 1023){
    result[2] = 1023;
  } else if (result[2] < 100){
    result[2] = 100;
  }
  if (result[3] > 1.5){
    result[3] = 1.5;
  } 
  if (result[7] > 100){ 
    result[7] = 100;
  } else if (result[7] < 1){
    result[7] = 1;
  }

// tiga
  if (result[4] > 1023){
    result[4] = 1023;
  } else if (result[4] < 100){
    result[4] = 100;
  }
  if (result[5] > 1.5){
    result[5] = 1.5;
  }
  if (result[8] > 100){
    result[8] = 100;
  } else if (result[8] < 1){
    result[8] = 1;
  }
  return result;
}

void actionMove(){
  JsonDocument doc1;
  data.target[0] = doc["targetInRad"]["rad1"][data.loopLen - 1].as<float>();
  data.target[1] = doc["targetInRad"]["rad2"][data.loopLen - 1].as<float>();
  data.target[2] = doc["targetInRad"]["rad3"][data.loopLen - 1].as<float>();

  for (int i = 0; i < 3; i++){
    data.current[0] = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE) - 150;
    data.current[1] = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
    data.current[2] = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);
    currentTime = millis();
    while(currentTime - previousTime < dataEventInterval){
      currentTime = millis();
    }
    previousTime = currentTime;
  }
  
  float* result = calcTorque();

  dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, result[0]);
  dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, result[2]);
  dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, result[4]);

  doc1["t1"] = result[1];
  doc1["t2"] = result[3];
  doc1["t3"] = result[5];
  
  if (data.theta[2] != data.prevTheta[2]){
    dxl.setGoalPosition(DXL_ID_THREE, data.theta[2], UNIT_DEGREE);
  }
  if (data.theta[1] != data.prevTheta[1]){
    dxl.setGoalPosition(DXL_ID_TWO, data.theta[1], UNIT_DEGREE);
  }
  if (data.theta[0] != data.prevTheta[0]){
    dxl.setGoalPosition(DXL_ID_ONE, data.theta[0], UNIT_DEGREE);
  }

  movingStatusCheck();
  
  if(doc["gripper"]){
   if (doc["gripper"]["buka"].as<signed int>() == data.loopLen) {
      gripper.startEaseTo(10);
      currentTime = millis();
      while(currentTime - previousTime < gripperEventInterval){
        currentTime = millis();
      }
      previousTime = currentTime;
     }
   if (doc["gripper"]["tutup"].as<signed int>() == data.loopLen) {
      gripper.startEaseTo(27);
      currentTime = millis();
      while(currentTime - previousTime < gripperEventInterval){
        currentTime = millis();
      }
      previousTime = currentTime;
   }
  }

  for (int i = 0; i < 3; i++){
    data.current[0] = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE) - 150;
    data.current[1] = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
    data.current[2] = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);
    currentTime = millis();
    while(currentTime - previousTime < dataEventInterval){
      currentTime = millis();
    }
    previousTime = currentTime;
  }

  float* resultFK = calcForwardKinematic();

  doc1["satu"] = result[6];
  doc1["dua"] = result[7];
  doc1["tiga"] = result[8];
  
  doc1["fk1"] = resultFK[0];
  doc1["fk2"] = resultFK[1];
  doc1["fk3"] = resultFK[2];
  
  if (data.loopLen == doc["thetaLen"].as<signed int>()){
    doc1["done"] = true;
  }

  serializeJson(doc1, DEBUG_SERIAL);
  doc1.clear();
}
void setup() {
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, 1000);
  dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, 512);
  dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, 512);
  gripper.attach(7, 10);
  gripper.setSpeed(200); 
  currentTime = millis();
  while(currentTime - previousTime < gripperEventInterval){
    currentTime = millis();
  }
  previousTime = currentTime;
  dxl.setGoalPosition(DXL_ID_ONE, 150, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_TWO, 100, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_THREE, 51, UNIT_DEGREE);
}

void loop() {
    if (DEBUG_SERIAL.available() > 0){
      String incomingData = DEBUG_SERIAL.readStringUntil('\n');
      DeserializationError err = deserializeJson(doc, incomingData);
      if (err) {
        DEBUG_SERIAL.print(F("deserializeJson() failed: "));
        DEBUG_SERIAL.println(err.f_str());
        return;
      }
      if (doc["sync"].as<String>() == "check") {
        Serial.print("sync_ok\n");
        doc.clear();
      } else {
        DEBUG_SERIAL.println("data ok");
        for(int i = 0; i < 6; i++) {
          gain1[i] = doc["matrixGain"]["satu"][i].as<float>();
          gain2[i] = doc["matrixGain"]["dua"][i].as<float>();
          gain3[i] = doc["matrixGain"]["tiga"][i].as<float>();
        }
        for(int i = 1; i <= doc["thetaLen"].as<signed int>(); i++){
          if (i == 1 ){
            data.theta[0] = doc["theta"]["satu"][i - 1].as<float>();
            data.theta[1] = doc["theta"]["dua"][i - 1].as<float>();
            data.theta[2] = doc["theta"]["tiga"][i - 1].as<float>();
            data.prevTheta[0] = 0;
            data.prevTheta[1] = 0;
            data.prevTheta[2] = 0;
          } else {
            data.theta[0] = doc["theta"]["satu"][i - 1].as<float>();
            data.theta[1] = doc["theta"]["dua"][i - 1].as<float>();
            data.theta[2] = doc["theta"]["tiga"][i - 1].as<float>();
            data.prevTheta[0] = doc["theta"]["satu"][i - 2].as<float>();
            data.prevTheta[1] = doc["theta"]["dua"][i - 2].as<float>();
            data.prevTheta[2] = doc["theta"]["tiga"][i - 2].as<float>();
          }       
          data.loopLen = i;
          actionMove();
        }
        doc.clear();
      }
    }
}