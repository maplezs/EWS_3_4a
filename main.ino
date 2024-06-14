#include <Dynamixel2Arduino.h>
#include <ArduinoJson.h>
#include <Servo.h>
JsonDocument doc;
#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
#define DEG_TO_RAD 0.017453292519943295769236907684886
const int DXL_DIR_PIN = 51; 
const int DXL_ID_ONE = 7; 
const int DXL_ID_TWO = 14; 
const int DXL_ID_THREE = 3; 
const float DXL_PROTOCOL_VERSION = 1.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;
float theta1, theta2, theta3;
Servo gripper;
double current1 = 0;
double current2 = 0;
double current3 = 0;
double target1 = 0;
double target2 = 0;
double target3 = 0;
double* calcTorque(double current1, double current2, double current3, double target1, double target2, double target3){
  static double result[3];
  double a[6] = {-0.0999999999999998, 0, 0, -0.170256693766065, 0, 0};
  double b[6] = {0, -0.389653589745392, -0.279653610881055, 0, -0.127515221024406, -0.0107280701401606};
  double c[6] = {0, -0.328416545719673, -0.417298516050250, 0, -0.0252016070810869, -0.111999712837656};

  current1 *= DEG_TO_RAD;
  current2 *= DEG_TO_RAD;
  current3 *= DEG_TO_RAD;

  double torsi1 = (-a[0] * target1) + (-a[1] * target2) + (-a[2] * target3) - (a[3] + a[4] + a[5]) * (target1 - current1);
  double torsi2 = (-b[0] * target1) + (-b[1] * target2) + (-b[2] * target3) - (b[3] + b[4] + b[5]) * (target2 - current2);
  double torsi3 = (-c[0] * target1) + (-c[1] * target2) + (-c[2] * target3) - (c[3] + c[4] + c[5]) * (target3 - current3);

  result[0] = abs(torsi1 * (1023 / 1.5));
  result[1] = abs(torsi1 * (1023 / 1.5));
  result[2] = abs(torsi1 * (1023 / 1.5));

  return result;
}
void setup() {
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, 767);
  dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, 767);
  dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, 767);
  gripper.attach(7);
  gripper.write(10);
  delay(1000);
  dxl.setGoalPosition(DXL_ID_ONE, 150.0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_TWO, 113.3275, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_THREE, 50.1099, UNIT_DEGREE);
}

void loop() {
  if (DEBUG_SERIAL.available() > 0){
    String incomingData = DEBUG_SERIAL.readStringUntil('\n');
    DeserializationError err = deserializeJson(doc, incomingData);
    if (err) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(err.f_str());
      return;
   } 
    target1 = doc["targetInRad"]["rad1"][0].as<double>();
    target2 = doc["targetInRad"]["rad2"][0].as<double>();
    target3 = doc["targetInRad"]["rad3"][0].as<double>();

    current1 = dxl.getPresentPosition(DXL_ID_ONE)  - 150;
    current2 = 190 + dxl.getPresentPosition(DXL_ID_TWO);
    current3 = 150 + dxl.getPresentPosition(DXL_ID_THREE);
    
    double* result = calcTorque(current1, current2, current3, target1, target2, target3); 

    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, result[0]);
    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, result[1]);
    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, result[2]);
    delay(100);
    Serial.println("SATU");
    Serial.println("=============================================");
    Serial.print(result[0]);Serial.print("\t");Serial.print(result[1]);Serial.print("\t");Serial.println(result[2]);
    theta1 = doc["finalTheta"]["satu"]["theta1"].as<float>();
    theta2 = doc["finalTheta"]["satu"]["theta2"].as<float>();
    theta3 = doc["finalTheta"]["satu"]["theta3"].as<float>();

    dxl.setGoalPosition(DXL_ID_ONE, theta1, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_TWO, theta2, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_THREE, theta3, UNIT_DEGREE);
    delay(2000);
    
    // DUA 
    target1 = doc["targetInRad"]["rad1"][1].as<double>();
    target2 = doc["targetInRad"]["rad2"][1].as<double>();
    target3 = doc["targetInRad"]["rad3"][1].as<double>();

    current1 = dxl.getPresentPosition(DXL_ID_ONE)  - 150;
    current2 = 190 + dxl.getPresentPosition(DXL_ID_TWO);
    current3 = 150 + dxl.getPresentPosition(DXL_ID_THREE);
    
    result = calcTorque(current1, current2, current3, target1, target2, target3); 

    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, result[0]);
    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, result[1]);
    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, result[2]);
    delay(100);
    Serial.println("DUA");
    Serial.println("=============================================");
    Serial.print(result[0]);Serial.print("\t");Serial.print(result[1]);Serial.print("\t");Serial.println(result[2]);
    theta1 = doc["finalTheta"]["dua"]["theta1"].as<float>();
    theta2 = doc["finalTheta"]["dua"]["theta2"].as<float>();
    theta3 = doc["finalTheta"]["dua"]["theta3"].as<float>();

    dxl.setGoalPosition(DXL_ID_ONE, theta1, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_TWO, theta2, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_THREE, theta3, UNIT_DEGREE);
    delay(2000);

    // TIGA
    target1 = doc["targetInRad"]["rad1"][2].as<double>();
    target2 = doc["targetInRad"]["rad2"][2].as<double>();
    target3 = doc["targetInRad"]["rad3"][2].as<double>();

    current1 = dxl.getPresentPosition(DXL_ID_ONE)  - 150;
    current2 = 190 + dxl.getPresentPosition(DXL_ID_TWO);
    current3 = 150 + dxl.getPresentPosition(DXL_ID_THREE);
    
    result = calcTorque(current1, current2, current3, target1, target2, target3); 

    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, result[0]);
    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, result[1]);
    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, result[2]);
    delay(100);
    gripper.write(140);
    delay(2500);
    Serial.println("TIGA");
    Serial.println("=============================================");
    Serial.print(result[0]);Serial.print("\t");Serial.print(result[1]);Serial.print("\t");Serial.println(result[2]);
    theta1 = doc["finalTheta"]["tiga"]["theta1"].as<float>();
    theta2 = doc["finalTheta"]["tiga"]["theta2"].as<float>();
    theta3 = doc["finalTheta"]["tiga"]["theta3"].as<float>();

    dxl.setGoalPosition(DXL_ID_ONE, theta1, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_TWO, theta2, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_THREE, theta3, UNIT_DEGREE);
    delay(2000);
    
    // EMPAT
    target1 = doc["targetInRad"]["rad1"][3].as<double>();
    target2 = doc["targetInRad"]["rad2"][3].as<double>();
    target3 = doc["targetInRad"]["rad3"][3].as<double>();

    current1 = dxl.getPresentPosition(DXL_ID_ONE)  - 150;
    current2 = 190 + dxl.getPresentPosition(DXL_ID_TWO);
    current3 = 150 + dxl.getPresentPosition(DXL_ID_THREE);
    
    result = calcTorque(current1, current2, current3, target1, target2, target3); 

    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, result[0]);
    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, result[1]);
    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, result[2]);
    delay(100);
    Serial.println("EMPAT");
    Serial.println("=============================================");
    Serial.print(result[0]);Serial.print("\t");Serial.print(result[1]);Serial.print("\t");Serial.println(result[2]);
    theta1 = doc["finalTheta"]["empat"]["theta1"].as<float>();
    theta2 = doc["finalTheta"]["empat"]["theta2"].as<float>();
    theta3 = doc["finalTheta"]["empat"]["theta3"].as<float>();

    dxl.setGoalPosition(DXL_ID_ONE, theta1, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_TWO, theta2, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_THREE, theta3, UNIT_DEGREE);
    delay(3000);
    
    // LIMA
    target1 = doc["targetInRad"]["rad1"][4].as<double>();
    target2 = doc["targetInRad"]["rad2"][4].as<double>();
    target3 = doc["targetInRad"]["rad3"][4].as<double>();

    current1 = dxl.getPresentPosition(DXL_ID_ONE)  - 150;
    current2 = 190 + dxl.getPresentPosition(DXL_ID_TWO);
    current3 = 150 + dxl.getPresentPosition(DXL_ID_THREE);
    
    result = calcTorque(current1, current2, current3, target1, target2, target3); 

    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, result[0]);
    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, result[1]);
    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, result[2]);
    delay(100);
    Serial.println("LIMA");
    Serial.println("=============================================");
    Serial.print(result[0]);Serial.print("\t");Serial.print(result[1]);Serial.print("\t");Serial.println(result[2]);
    theta1 = doc["finalTheta"]["lima"]["theta1"].as<float>();
    theta2 = doc["finalTheta"]["lima"]["theta2"].as<float>();
    theta3 = doc["finalTheta"]["lima"]["theta3"].as<float>();

    dxl.setGoalPosition(DXL_ID_ONE, theta1, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_TWO, theta2, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_THREE, theta3, UNIT_DEGREE);
    delay(2000);
    
    // ENAM
    target1 = doc["targetInRad"]["rad1"][5].as<double>();
    target2 = doc["targetInRad"]["rad2"][5].as<double>();
    target3 = doc["targetInRad"]["rad3"][5].as<double>();

    current1 = dxl.getPresentPosition(DXL_ID_ONE)  - 150;
    current2 = 190 + dxl.getPresentPosition(DXL_ID_TWO);
    current3 = 150 + dxl.getPresentPosition(DXL_ID_THREE);
    
    result = calcTorque(current1, current2, current3, target1, target2, target3); 

    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, result[0]);
    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, result[1]);
    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, result[2]);
    delay(100);
    gripper.write(10);
    delay(2000);
    Serial.println("ENAM");
    Serial.println("=============================================");
    Serial.print(result[0]);Serial.print("\t");Serial.print(result[1]);Serial.print("\t");Serial.println(result[2]);
    theta1 = doc["finalTheta"]["enam"]["theta1"].as<float>();
    theta2 = doc["finalTheta"]["enam"]["theta2"].as<float>();
    theta3 = doc["finalTheta"]["enam"]["theta3"].as<float>();

    dxl.setGoalPosition(DXL_ID_ONE, theta1, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_TWO, theta2, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_THREE, theta3, UNIT_DEGREE);
    delay(2000);
    
    // TUJUH
    target1 = doc["targetInRad"]["rad1"][6].as<double>();
    target2 = doc["targetInRad"]["rad2"][6].as<double>();
    target3 = doc["targetInRad"]["rad3"][6].as<double>();

    current1 = dxl.getPresentPosition(DXL_ID_ONE)  - 150;
    current2 = 190 + dxl.getPresentPosition(DXL_ID_TWO);
    current3 = 150 + dxl.getPresentPosition(DXL_ID_THREE);
    
    result = calcTorque(current1, current2, current3, target1, target2, target3); 

    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_ONE, result[0]);
    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_TWO, result[1]);
    dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID_THREE, result[2]);
    delay(100);
    Serial.println("TUJUH");
    Serial.println("=============================================");
    Serial.print(result[0]);Serial.print("\t");Serial.print(result[1]);Serial.print("\t");Serial.println(result[2]);

    theta1 = doc["finalTheta"]["tujuh"]["theta1"].as<float>();
    theta2 = doc["finalTheta"]["tujuh"]["theta2"].as<float>();
    theta3 = doc["finalTheta"]["tujuh"]["theta3"].as<float>();

    dxl.setGoalPosition(DXL_ID_ONE, theta1, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_TWO, theta2, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_THREE, theta3, UNIT_DEGREE);
  }
}
