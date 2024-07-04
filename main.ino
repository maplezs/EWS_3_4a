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
float theta1, theta2, theta3;
Servo gripper;
float current1 = 0;
float current2 = 0;
float current3 = 0;
float target1 = 0;
float target2 = 0;
float target3 = 0;
float gain1[6];
float gain2[6];
float gain3[6];
bool start = false;
uint16_t max_torque_one = 0;
uint16_t max_torque_two = 0;
uint16_t max_torque_three = 0;

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

float* calcForwardKinematic(float current1, float current2, float current3) {
  static float resultFK[3];
  current1 *= DEG_TO_RAD;
  current2 *= DEG_TO_RAD;
  current3 *= DEG_TO_RAD;
  float l1 = 0.125;
  float l2 = 0.15;
  float l3 = 0.18;
  float x = (l3*cos(current2-current3)+l2*cos(current2))*cos(current1);
  float y = (l3*cos(current2-current3)+l2*cos(current2))*sin(current1);
  float z = l3*sin(current2-current3)+l2*sin(current2)+l1;
  resultFK[0] = abs(x);
  resultFK[1] = y;
  resultFK[2] = abs(z);
  
  return resultFK;
}

float* calcTorque(float current1, float current2, float current3, float target1, float target2, float target3) {
  static float result[6];
  current1 *= DEG_TO_RAD;
  current2 *= DEG_TO_RAD;
  current3 *= DEG_TO_RAD;

  float torsi1 = (-gain1[0] * target1) + (-gain1[1] * target2) + (-gain1[2] * target3) + (-gain1[3] * (target1 - current1)) + (-gain1[4] * (target2 - current2)) + (-gain1[5] * (target3 - current3));
  float torsi2 = (-gain2[0] * target1) + (-gain2[1] * target2) + (-gain2[2] * target3) + (-gain2[3] * (target1 - current1)) + (-gain2[4] * (target2 - current2)) + (-gain2[5] * (target3 - current3));
  float torsi3 = (-gain3[0] * target1) + (-gain3[1] * target2) + (-gain3[2] * target3) + (-gain3[3] * (target1 - current1)) + (-gain3[4] * (target2 - current2)) + (-gain3[5] * (target3 - current3));
  
  result[0] = ceil(abs(torsi1 * (1023 / 1.5)));
  result[1] = abs(torsi1);
  result[2] = ceil(abs(torsi2 * (1023 / 1.5)));
  result[3] = abs(torsi2);
  result[4] = ceil(abs(torsi3 * (1023 / 1.5)));
  result[5] = abs(torsi3);

// satu
  if (result[0] > 1023){
    result[0] = 1023;
  } else if (result[0] < 100){
    result[0] = 100;
  }

// dua 
  if (result[2] > 1023){
    result[2] = 1023;
  } else if (result[2] < 100){
    result[2] = 100;
  } 

// tiga
  if (result[4] > 1023){
    result[4] = 1023;
  } else if (result[4] < 100){
    result[4] = 100;
  }

  return result;
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
    JsonDocument doc;
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
        DEBUG_SERIAL.println("data 1 ok");
        start = true;
      }
      while(start){
        if (DEBUG_SERIAL.available() > 0){
        JsonDocument doc2;
        JsonDocument doc3;
        String incomingData2 = DEBUG_SERIAL.readStringUntil('\n');
        DeserializationError err2 = deserializeJson(doc2, incomingData2);
          if (err2) {
            DEBUG_SERIAL.print(F("deserializeJson() failed: "));
            DEBUG_SERIAL.println(err.f_str());
            return;
          }
        DEBUG_SERIAL.println("data 2 ok");
      for(int i = 0; i <=5; i++) {
        gain1[i] = doc2["matrixGain"]["satu"][i].as<float>();
        gain2[i] = doc2["matrixGain"]["dua"][i].as<float>();
        gain3[i] = doc2["matrixGain"]["tiga"][i].as<float>();
      }
      delay(100);
      // SATU
      target1 = doc2["targetInRad"]["rad1"][0].as<float>();
      target2 = doc2["targetInRad"]["rad2"][0].as<float>();
      target3 = doc2["targetInRad"]["rad3"][0].as<float>();

      current1 = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE)  - 150;
      current2 = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
      current3 = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);
      
      float* result = calcTorque(current1, current2, current3, target1, target2, target3); 

      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, result[0]);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, result[2]);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, result[4]);
      
      dxl.read(DXL_ID_ONE, 34, 2, (uint8_t*)&max_torque_one, sizeof(max_torque_one), 10);
      dxl.read(DXL_ID_TWO, 34, 2, (uint8_t*)&max_torque_two, sizeof(max_torque_two), 10);
      dxl.read(DXL_ID_THREE, 34, 2, (uint8_t*)&max_torque_three, sizeof(max_torque_three), 10);
      doc3["satu"] = max_torque_one;
      doc3["dua"] = max_torque_two;
      doc3["tiga"] = max_torque_three;

      doc3["t1"] = result[1];
      doc3["t2"] = result[3];
      doc3["t3"] = result[5];
      
      theta1 = doc["finalTheta"]["satu"]["theta1"].as<float>();
      theta2 = doc["finalTheta"]["satu"]["theta2"].as<float>();
      theta3 = doc["finalTheta"]["satu"]["theta3"].as<float>();

      dxl.setGoalPosition(DXL_ID_ONE, theta1, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_TWO, theta2, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_THREE, theta3, UNIT_DEGREE);
      movingStatusCheck(result[2], result[4], theta2, theta3);

      current1 = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE)  - 150;
      current2 = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
      current3 = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);
      float* resultFK = calcForwardKinematic(current1, current2, current3);
      doc3["fk1"] = resultFK[0];
      doc3["fk2"] = resultFK[1];
      doc3["fk3"] = resultFK[2];
      serializeJson(doc3, DEBUG_SERIAL);
      

      // DUA 
      target1 = doc2["targetInRad"]["rad1"][1].as<float>();
      target2 = doc2["targetInRad"]["rad2"][1].as<float>();
      target3 = doc2["targetInRad"]["rad3"][1].as<float>();

      current1 = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE)  - 150;
      current2 = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
      current3 = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);

      result = calcTorque(current1, current2, current3, target1, target2, target3); 

      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, result[0]);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, result[2]);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, result[4]);
      if (doc2["gripper"]["satu"]) {
        gripperAction(doc2["gripper"]["satu"].as<String>());
      }
      doc3.clear();
      dxl.read(DXL_ID_ONE, 34, 2, (uint8_t*)&max_torque_one, sizeof(max_torque_one), 10);
      dxl.read(DXL_ID_TWO, 34, 2, (uint8_t*)&max_torque_two, sizeof(max_torque_two), 10);
      dxl.read(DXL_ID_THREE, 34, 2, (uint8_t*)&max_torque_three, sizeof(max_torque_three), 10);
      doc3["satu"] = max_torque_one;
      doc3["dua"] = max_torque_two;
      doc3["tiga"] = max_torque_three;

      doc3["t1"] = result[1];
      doc3["t2"] = result[3];
      doc3["t3"] = result[5];

      theta1 = doc["finalTheta"]["dua"]["theta1"].as<float>();
      theta2 = doc["finalTheta"]["dua"]["theta2"].as<float>();
      theta3 = doc["finalTheta"]["dua"]["theta3"].as<float>();

      dxl.setGoalPosition(DXL_ID_ONE, theta1, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_TWO, theta2, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_THREE, theta3, UNIT_DEGREE);
      movingStatusCheck(result[2], result[4], theta2, theta3);

      current1 = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE)  - 150;
      current2 = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
      current3 = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);
      resultFK = calcForwardKinematic(current1, current2, current3);
      doc3["fk1"] = resultFK[0];
      doc3["fk2"] = resultFK[1];
      doc3["fk3"] = resultFK[2];
      serializeJson(doc3, DEBUG_SERIAL);
      

      // TIGA
      target1 = doc2["targetInRad"]["rad1"][2].as<float>();
      target2 = doc2["targetInRad"]["rad2"][2].as<float>();
      target3 = doc2["targetInRad"]["rad3"][2].as<float>();

      current1 = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE)  - 150;
      current2 = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
      current3 = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);

      result = calcTorque(current1, current2, current3, target1, target2, target3); 

      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, result[0]);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, result[2]);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, result[4]);
      if (doc2["gripper"]["dua"]) {
        gripperAction(doc2["gripper"]["dua"].as<String>());
      }
      doc3.clear();
      dxl.read(DXL_ID_ONE, 34, 2, (uint8_t*)&max_torque_one, sizeof(max_torque_one), 10);
      dxl.read(DXL_ID_TWO, 34, 2, (uint8_t*)&max_torque_two, sizeof(max_torque_two), 10);
      dxl.read(DXL_ID_THREE, 34, 2, (uint8_t*)&max_torque_three, sizeof(max_torque_three), 10);
      doc3["satu"] = max_torque_one;
      doc3["dua"] = max_torque_two;
      doc3["tiga"] = max_torque_three;

      doc3["t1"] = result[1];
      doc3["t2"] = result[3];
      doc3["t3"] = result[5];

      theta1 = doc["finalTheta"]["tiga"]["theta1"].as<float>();
      theta2 = doc["finalTheta"]["tiga"]["theta2"].as<float>();
      theta3 = doc["finalTheta"]["tiga"]["theta3"].as<float>();

      dxl.setGoalPosition(DXL_ID_ONE, theta1, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_TWO, theta2, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_THREE, theta3, UNIT_DEGREE);
      movingStatusCheck(result[2], result[4], theta2, theta3);
      
      current1 = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE)  - 150;
      current2 = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
      current3 = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);
      resultFK = calcForwardKinematic(current1, current2, current3);
      doc3["fk1"] = resultFK[0];
      doc3["fk2"] = resultFK[1];
      doc3["fk3"] = resultFK[2];
      serializeJson(doc3, DEBUG_SERIAL);
      

      // EMPAT
      target1 = doc2["targetInRad"]["rad1"][3].as<float>();
      target2 = doc2["targetInRad"]["rad2"][3].as<float>();
      target3 = doc2["targetInRad"]["rad3"][3].as<float>();

      current1 = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE)  - 150;
      current2 = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
      current3 = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);

      result = calcTorque(current1, current2, current3, target1, target2, target3); 

      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, result[0]);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, result[2]);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, result[4]);
      if (doc2["gripper"]["tiga"]) {
        gripperAction(doc2["gripper"]["tiga"].as<String>());
      }
      doc3.clear();
      dxl.read(DXL_ID_ONE, 34, 2, (uint8_t*)&max_torque_one, sizeof(max_torque_one), 10);
      dxl.read(DXL_ID_TWO, 34, 2, (uint8_t*)&max_torque_two, sizeof(max_torque_two), 10);
      dxl.read(DXL_ID_THREE, 34, 2, (uint8_t*)&max_torque_three, sizeof(max_torque_three), 10);
      doc3["satu"] = max_torque_one;
      doc3["dua"] = max_torque_two;
      doc3["tiga"] = max_torque_three;

      doc3["t1"] = result[1];
      doc3["t2"] = result[3];
      doc3["t3"] = result[5];

      theta1 = doc["finalTheta"]["empat"]["theta1"].as<float>();
      theta2 = doc["finalTheta"]["empat"]["theta2"].as<float>();
      theta3 = doc["finalTheta"]["empat"]["theta3"].as<float>();
      
      dxl.setGoalPosition(DXL_ID_ONE, theta1, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_TWO, theta2, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_THREE, theta3, UNIT_DEGREE);
      movingStatusCheck(result[2], result[4], theta2, theta3);   
      
      current1 = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE)  - 150;
      current2 = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
      current3 = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);
      resultFK = calcForwardKinematic(current1, current2, current3);
      doc3["fk1"] = resultFK[0];
      doc3["fk2"] = resultFK[1];
      doc3["fk3"] = resultFK[2];
      serializeJson(doc3, DEBUG_SERIAL);
      

      // LIMA
      target1 = doc2["targetInRad"]["rad1"][4].as<float>();
      target2 = doc2["targetInRad"]["rad2"][4].as<float>();
      target3 = doc2["targetInRad"]["rad3"][4].as<float>();

      current1 = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE)  - 150;
      current2 = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
      current3 = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);
      result = calcTorque(current1, current2, current3, target1, target2, target3); 

      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, result[0]);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, result[2]);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, result[4]);
      if (doc2["gripper"]["empat"]) {
        gripperAction(doc2["gripper"]["empat"].as<String>());
      }
      doc3.clear();
      dxl.read(DXL_ID_ONE, 34, 2, (uint8_t*)&max_torque_one, sizeof(max_torque_one), 10);
      dxl.read(DXL_ID_TWO, 34, 2, (uint8_t*)&max_torque_two, sizeof(max_torque_two), 10);
      dxl.read(DXL_ID_THREE, 34, 2, (uint8_t*)&max_torque_three, sizeof(max_torque_three), 10);
      doc3["satu"] = max_torque_one;
      doc3["dua"] = max_torque_two;
      doc3["tiga"] = max_torque_three;

      doc3["t1"] = result[1];
      doc3["t2"] = result[3];
      doc3["t3"] = result[5];

      theta1 = doc["finalTheta"]["lima"]["theta1"].as<float>();
      theta2 = doc["finalTheta"]["lima"]["theta2"].as<float>();
      theta3 = doc["finalTheta"]["lima"]["theta3"].as<float>();

      dxl.setGoalPosition(DXL_ID_ONE, theta1, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_TWO, theta2, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_THREE, theta3, UNIT_DEGREE);
      movingStatusCheck(result[2], result[4], theta2, theta3);

      current1 = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE)  - 150;
      current2 = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
      current3 = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);
      resultFK = calcForwardKinematic(current1, current2, current3);
      doc3["fk1"] = resultFK[0];
      doc3["fk2"] = resultFK[1];
      doc3["fk3"] = resultFK[2];
      serializeJson(doc3, DEBUG_SERIAL);
      

      // ENAM
      target1 = doc2["targetInRad"]["rad1"][5].as<float>();
      target2 = doc2["targetInRad"]["rad2"][5].as<float>();
      target3 = doc2["targetInRad"]["rad3"][5].as<float>();

      current1 = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE)  - 150;
      current2 = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
      current3 = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);

      result = calcTorque(current1, current2, current3, target1, target2, target3); 

      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, result[0]);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, result[2]);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, result[4]);
      if (doc2["gripper"]["lima"]) {
        gripperAction(doc2["gripper"]["lima"].as<String>());
      }
      doc3.clear();
      dxl.read(DXL_ID_ONE, 34, 2, (uint8_t*)&max_torque_one, sizeof(max_torque_one), 10);
      dxl.read(DXL_ID_TWO, 34, 2, (uint8_t*)&max_torque_two, sizeof(max_torque_two), 10);
      dxl.read(DXL_ID_THREE, 34, 2, (uint8_t*)&max_torque_three, sizeof(max_torque_three), 10);
      doc3["satu"] = max_torque_one;
      doc3["dua"] = max_torque_two;
      doc3["tiga"] = max_torque_three;

      doc3["t1"] = result[1];
      doc3["t2"] = result[3];
      doc3["t3"] = result[5];

      theta1 = doc["finalTheta"]["enam"]["theta1"].as<float>();
      theta2 = doc["finalTheta"]["enam"]["theta2"].as<float>();
      theta3 = doc["finalTheta"]["enam"]["theta3"].as<float>();

      dxl.setGoalPosition(DXL_ID_ONE, theta1, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_TWO, theta2, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_THREE, theta3, UNIT_DEGREE);
      movingStatusCheck(result[2], result[4], theta2, theta3);
      
      current1 = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE)  - 150;
      current2 = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
      current3 = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);
      resultFK = calcForwardKinematic(current1, current2, current3);
      doc3["fk1"] = resultFK[0];
      doc3["fk2"] = resultFK[1];
      doc3["fk3"] = resultFK[2];
      serializeJson(doc3, DEBUG_SERIAL);
      

      // TUJUH
      target1 = doc2["targetInRad"]["rad1"][6].as<float>();
      target2 = doc2["targetInRad"]["rad2"][6].as<float>();
      target3 = doc2["targetInRad"]["rad3"][6].as<float>();

      current1 = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE)  - 150;
      current2 = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
      current3 = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);

      result = calcTorque(current1, current2, current3, target1, target2, target3); 

      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, result[0]);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, result[2]);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, result[4]);
      if (doc2["gripper"]["enam"]) {
        gripperAction(doc2["gripper"]["enam"].as<String>());
      }
      doc3.clear();
      dxl.read(DXL_ID_ONE, 34, 2, (uint8_t*)&max_torque_one, sizeof(max_torque_one), 10);
      dxl.read(DXL_ID_TWO, 34, 2, (uint8_t*)&max_torque_two, sizeof(max_torque_two), 10);
      dxl.read(DXL_ID_THREE, 34, 2, (uint8_t*)&max_torque_three, sizeof(max_torque_three), 10);
      doc3["satu"] = max_torque_one;
      doc3["dua"] = max_torque_two;
      doc3["tiga"] = max_torque_three;
      doc3["t1"] = result[1];
      doc3["t2"] = result[3];
      doc3["t3"] = result[5];

      theta1 = doc["finalTheta"]["tujuh"]["theta1"].as<float>();
      theta2 = doc["finalTheta"]["tujuh"]["theta2"].as<float>();
      theta3 = doc["finalTheta"]["tujuh"]["theta3"].as<float>();

      dxl.setGoalPosition(DXL_ID_ONE, theta1, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_TWO, theta2, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_THREE, theta3, UNIT_DEGREE);
      movingStatusCheck(result[2], result[4], theta2, theta3);

      current1 = dxl.getPresentPosition(DXL_ID_ONE, UNIT_DEGREE)  - 150;
      current2 = 190 - dxl.getPresentPosition(DXL_ID_TWO, UNIT_DEGREE);
      current3 = 150 - dxl.getPresentPosition(DXL_ID_THREE, UNIT_DEGREE);
      resultFK = calcForwardKinematic(current1, current2, current3);
      doc3["fk1"] = resultFK[0];
      doc3["fk2"] = resultFK[1];
      doc3["fk3"] = resultFK[2];
      doc3["done"] = true;
      serializeJson(doc3, DEBUG_SERIAL);
      
      if (doc2["gripper"]["tujuh"]) {
        gripperAction(doc2["gripper"]["tujuh"].as<String>());
      }
      start = false;
        }
      }
    }
}