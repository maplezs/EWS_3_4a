#include <Dynamixel2Arduino.h>
#include <ArduinoJson.h>
#include <Servo.h>
#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
#define DEG_TO_RAD 0.017453292519943295769236907684886
const int DXL_DIR_PIN = 51; 
const int DXL_ID_ONE = 7; 
const int DXL_ID_TWO = 2; 
const int DXL_ID_THREE = 3; 
const float DXL_PROTOCOL_VERSION = 1.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;
Servo gripper;
float gain1[6];
float gain2[6];
float gain3[6];
JsonDocument doc;
struct movingData {
  float theta[3];
  float current[3];
  float target[3];
  uint16_t readTorque[3];
  int loopLen;
};
void gripperAction(String action){
  if (action == "buka") {
    gripper.write(10);
    delay(1000);
  } else {
    gripper.write(120);
    delay(1000);
  }
}

void movingStatusCheck(float torque_2, float torque_3, float goal_2, float goal_3) {
  int movingStatus = 1;
  uint8_t moving_returned_one;
  uint8_t moving_returned_two;
  uint8_t moving_returned_three;
  while (movingStatus) {
    dxl.read(DXL_ID_ONE, 46, 1, (uint8_t*)&moving_returned_one, sizeof(moving_returned_one), 10);
    dxl.read(DXL_ID_TWO, 46, 1, (uint8_t*)&moving_returned_two, sizeof(moving_returned_two), 10);
    dxl.read(DXL_ID_THREE, 46, 1, (uint8_t*)&moving_returned_three, sizeof(moving_returned_three), 10);
    if (!moving_returned_one && !moving_returned_two && !moving_returned_three) {
      movingStatus = 0;
      delay(500);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, torque_2);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, torque_3);
      delay(100);
      dxl.setGoalPosition(DXL_ID_TWO, goal_2, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_THREE, goal_3, UNIT_DEGREE);
    }
  }
}

float* calcForwardKinematic(movingData& data) {
  static float resultFK[3];
  data.current[0] *= DEG_TO_RAD;
  data.current[1] *= DEG_TO_RAD;
  data.current[2] *= DEG_TO_RAD;
  float l1 = 0.145;
  float l2 = 0.15;
  float l3 = 0.18;
  float x = (l3*cos(data.current[1]-data.current[2])+l2*cos(data.current[1]))*cos(data.current[0]);
  float y = (l3*cos(data.current[1]-data.current[2])+l2*cos(data.current[1]))*sin(data.current[0]);
  float z = l3*sin(data.current[1]-data.current[2])+l2*sin(data.current[1])+l1;
  resultFK[0] = abs(x);
  resultFK[1] = y;
  resultFK[2] = abs(z);
  
  return resultFK;
}

float* calcTorque(movingData& data) {
  static float result[9];
  data.current[0] *= DEG_TO_RAD;
  data.current[1] *= DEG_TO_RAD;
  data.current[2] *= DEG_TO_RAD;

  float torsi1 = (-gain1[0] * data.target[0]) + (-gain1[1] * data.target[1]) + (-gain1[2] * data.target[2]) + (-gain1[3] * (data.target[0] - data.current[0])) + (-gain1[4] * (data.target[1] - data.current[1])) + (-gain1[5] * (data.target[2] - data.current[2]));
  float torsi2 = (-gain2[0] * data.target[0]) + (-gain2[1] * data.target[1]) + (-gain2[2] * data.target[2]) + (-gain2[3] * (data.target[0] - data.current[0])) + (-gain2[4] * (data.target[1] - data.current[1])) + (-gain2[5] * (data.target[2] - data.current[2]));
  float torsi3 = (-gain3[0] * data.target[0]) + (-gain3[1] * data.target[1]) + (-gain3[2] * data.target[2]) + (-gain3[3] * (data.target[0] - data.current[0])) + (-gain3[4] * (data.target[1] - data.current[1])) + (-gain3[5] * (data.target[2] - data.current[2]));
  
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
  if (result[8] > 100){
    result[8] = 100;
  } else if (result[8] < 1){
    result[8] = 1;
  }
  return result;
}
void actionMove(movingData& data){
  JsonDocument doc1;
  data.target[0] = doc["targetInRad"]["rad1"][data.loopLen].as<float>();
  data.target[1] = doc["targetInRad"]["rad2"][data.loopLen].as<float>();
  data.target[2] = doc["targetInRad"]["rad3"][data.loopLen].as<float>();

  data.current[0] = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE)  - 150;
  data.current[1] = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
  data.current[2] = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);
  
  float* result = calcTorque(data);

  dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, result[0]);
  dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, result[2]);
  dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, result[4]);

  if(doc["gripper"]){
    if (doc["gripper"]["buka"].as<signed int>() == data.loopLen) {
        gripperAction("buka");
    }
    if (doc["gripper"]["tutup"].as<signed int>() == data.loopLen) {
          gripperAction("tutup");
    }
  }

  dxl.read(DXL_ID_ONE, 34, 2, (uint8_t*)&data.readTorque[0], sizeof(data.readTorque[0]), 10);
  dxl.read(DXL_ID_TWO, 34, 2, (uint8_t*)&data.readTorque[1], sizeof(data.readTorque[1]), 10);
  dxl.read(DXL_ID_THREE, 34, 2, (uint8_t*)&data.readTorque[2], sizeof(data.readTorque[2]), 10);

  doc1["satu"] = data.readTorque[0];
  doc1["dua"] = data.readTorque[1];
  doc1["tiga"] = data.readTorque[2];

  doc1["t1"] = result[1];
  doc1["t2"] = result[3];
  doc1["t3"] = result[5];
  
  dxl.setGoalPosition(DXL_ID_ONE, data.theta[0], UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_TWO, data.theta[1], UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_THREE, data.theta[2], UNIT_DEGREE);

  movingStatusCheck(result[2], result[4], data.theta[1], data.theta[2]);

  data.current[0] = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE)  - 150;
  data.current[1] = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
  data.current[2] = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);

  float* resultFK = calcForwardKinematic(data);

  doc1["fk1"] = resultFK[0];
  doc1["fk2"] = resultFK[1];
  doc1["fk3"] = resultFK[2];
  if (data.loopLen == doc["thetaLen"].as<signed int>() - 1){
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
  dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, 512);
  dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, 512);
  dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, 512);
  gripper.attach(7);
  gripper.write(10);
  delay(1000);
  dxl.setGoalPosition(DXL_ID_ONE, 150.0, UNIT_DEGREE);
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
        movingData data;
        for(int i = 0; i < doc["thetaLen"].as<signed int>(); i++){
          data.theta[0] = doc["theta"]["satu"][i].as<float>();
          data.theta[1] = doc["theta"]["dua"][i].as<float>();
          data.theta[2] = doc["theta"]["tiga"][i].as<float>();
          data.loopLen = i;
          actionMove(data);
        }
        doc.clear();
      }
    }
}