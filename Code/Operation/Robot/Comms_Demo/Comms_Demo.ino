/*
References used throughout this project (across both PC and Arduino):
- The bleak library for BLE on PC: https://github.com/hbldh/bleak/tree/develop
- The ESP32 BLE library: https://github.com/nkolban/ESP32_BLE_Arduino/tree/master
- The UUID generator is helpful: https://www.uuidgenerator.net/
- The XInput library for using a controller: https://pypi.org/project/XInput-Python/
*/

#include "../../Robot/Helpers/RobotBLE.cpp"
#include "../../Robot/Helpers/data.cpp"

// #define DEBUG

#define ROBOT_ID 0

#define SEND_DELAY 500 // milliseconds of delay time to wait before sending robot status

String robot_name = "NSFFM_" + String(ROBOT_ID);

data_positions data_positions_obj;
data_robot data_robot_obj;
data_command data_command_obj;

RobotBLE ble(robot_name, &data_positions_obj, &data_robot_obj, &data_command_obj); 

unsigned long start_time = 0;         
unsigned long current_time = 0; 
unsigned long last_send_time = 0; 

void setup() {
  Serial.begin(921600);

  #ifdef DEBUG
  Serial.println("Beginning BLE startup.");
  #endif
  ble.setup();
  #ifdef DEBUG
  Serial.println("Finished BLE startup.");
  #endif

  randomSeed(ROBOT_ID);

  start_time = millis();
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MAIN LOOP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void loop() {
  current_time = millis();
  if (current_time - last_send_time > SEND_DELAY) {
    Serial.println("SENDING DATA");
    last_send_time = current_time;
    float time = 0.001*(float)(current_time - start_time);

    data_robot_obj.num = ROBOT_ID;
    data_robot_obj.time = time;

    ble.send_robot();
  }

  // Updating state if there is an available measurement/command from wireless
  if (ble.check_update_positions()) {
    char tag_ID = 0;
    float pos_x = data_positions_obj.get_pos_x(tag_ID);
    float pos_y = data_positions_obj.get_pos_y(tag_ID);
    float theta = data_positions_obj.get_theta(tag_ID);
    Serial.print("TAG: ");    Serial.print(tag_ID,1); 
    Serial.print(", X:");     Serial.print(pos_x,5); 
    Serial.print(", Y:");     Serial.print(pos_y,5); 
    Serial.print(", THETA:"); Serial.print(theta,5); 
    Serial.println("");
  }

  // Updating state if there is an available measurement/command from wireless
  if (ble.check_update_command()) {
    Serial.print("NUM: ");         Serial.print(data_command_obj.num,1);
    Serial.print("AUTONOMOUS: ");  Serial.print(data_command_obj.autonomous);
    Serial.print("OSCILLATION: "); Serial.print(data_command_obj.oscillation);
    Serial.print("ACCESSORY: ");   Serial.print(data_command_obj.accessory);
    Serial.print("RESET: ");       Serial.print(data_command_obj.reset);
    Serial.print("MODE: ");        Serial.print(data_command_obj.mode);
    Serial.print("LEFT: ");        Serial.print(data_command_obj.left,5); 
    Serial.print("RIGHT: ");       Serial.print(data_command_obj.right,5); 
    Serial.println("");
  }

  delayMicroseconds(10000);
}