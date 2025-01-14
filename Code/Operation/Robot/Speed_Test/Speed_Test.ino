/*
References used throughout this project (across both PC and Arduino):
- The bleak library for BLE on PC: https://github.com/hbldh/bleak/tree/develop
- The ESP32 BLE library: https://github.com/nkolban/ESP32_BLE_Arduino/tree/master
- The UUID generator is helpful: https://www.uuidgenerator.net/
- The XInput library for using a controller: https://pypi.org/project/XInput-Python/
*/

#include "../../Common/constants.hpp"
#include "../../Common/ErgodicControl.cpp"
#include "../../Robot/Helpers/RobotBLE.cpp"
#include "../../Robot/Helpers/Robot.cpp"
#include "../../Robot/Helpers/data.cpp"

#define DEBUG

#define ROBOT_ID 1

#define EPSILON 0.01

String robot_name = "NSFFM_" + String(ROBOT_ID);

// Timing stuff
unsigned long start_time = 0;            // Time in millis when the setup ended
unsigned long current_time = 0;          // Time in millis at beginning of current main loop
unsigned long last_control_time = 0;     // Time in millis when the controls were last updated
float control_dt = SIM_DT;               // Time in seconds since last control update

// Data structures
data_robot data_robot_obj;

// Important objects
ErgodicControl controller;

void setup() {
  Serial.begin(921600);

  data_robot_obj.num = ROBOT_ID;
  data_robot_obj.pos_x = X_N_0;
  data_robot_obj.pos_y = Y_N_0;
  data_robot_obj.theta = TH_N_0;
  data_robot_obj.time = 0.0;

  #ifdef DEBUG
  Serial.println("Beginning controller startup.");
  #endif
  controller.Startup();
  #ifdef DEBUG
  Serial.println("Finished controller startup.");
  #endif

  randomSeed(ROBOT_ID);

  start_time = millis();
}


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MAIN LOOP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void loop() {
  current_time = millis();
  #ifdef DEBUG
  Serial.print("Starting control loop after ");
  Serial.print(current_time - last_control_time);
  Serial.println(" milliseconds.");
  #endif
  last_control_time = current_time;

  // --- Update the control horizon --- // 
  controller.UpdateControlHorizon(data_robot_obj.pos_x,
                                  data_robot_obj.pos_y,
                                  data_robot_obj.theta,
                                  data_robot_obj.time); // Generate controls using the last state and statistics
  data_robot_obj.v_left = controller.GetControlLeft(0); 
  if      (data_robot_obj.v_left < -U_N_LIM) {data_robot_obj.v_left = -U_N_LIM; }
  else if (data_robot_obj.v_left >  U_N_LIM) {data_robot_obj.v_left =  U_N_LIM; }
  data_robot_obj.v_right = controller.GetControlRight(0);
  if      (data_robot_obj.v_right < -U_N_LIM) {data_robot_obj.v_right = -U_N_LIM; }
  else if (data_robot_obj.v_right >  U_N_LIM) {data_robot_obj.v_right =  U_N_LIM; }

  // --- Predict the change in state (dead reckoning upated occasionally by camera) --- //
  data_robot_obj.pos_x = data_robot_obj.pos_x + control_dt*cosf(data_robot_obj.theta)*(data_robot_obj.v_left+data_robot_obj.v_right)*0.5;
  if      (data_robot_obj.pos_x < EPSILON)   {data_robot_obj.pos_x = EPSILON; }
  else if (data_robot_obj.pos_x > 1-EPSILON) {data_robot_obj.pos_x = 1-EPSILON; }
  data_robot_obj.pos_y = data_robot_obj.pos_y + control_dt*sinf(data_robot_obj.theta)*(data_robot_obj.v_left+data_robot_obj.v_right)*0.5;
  if      (data_robot_obj.pos_y < EPSILON)   {data_robot_obj.pos_y = EPSILON; }
  else if (data_robot_obj.pos_y > 1-EPSILON) {data_robot_obj.pos_y = 1-EPSILON; }
  data_robot_obj.theta = data_robot_obj.theta + control_dt*(-data_robot_obj.v_left+data_robot_obj.v_right)/S_N;
  data_robot_obj.time += control_dt; 

  controller.UpdateResultDistribution(data_robot_obj.pos_x,
                                      data_robot_obj.pos_y,
                                      data_robot_obj.theta,
                                      data_robot_obj.time);
}




// // Interrupt function that runs when the top sensor is pressed
// void ts_handler() {
//   if (stopAtTop) {
//     robot.setOscillationFrequency(0.0);
//   }
// }

// attachInterrupt(digitalPinToInterrupt(TS), ts_handler, RISING);

// --- OLD VERSION that did not use random heading --- //
        // if (robot.readBS_Q1()) {      // Turn CCW 135 degrees if moving forward, CW 45 degrees if moving backward
        //   Serial.println("Bounce Q1!");
        //   if (forward) {
        //     robot.setSpeedLeft(-turn_velocity);
        //     robot.setSpeedRight(turn_velocity);
        //     bounce_duration = turn_135_time;
        //   }
        //   else {
        //     robot.setSpeedLeft(turn_velocity);
        //     robot.setSpeedRight(-turn_velocity);
        //     bounce_duration = turn_45_time;
        //   }
        //   bounce_start_time = millis();
        //   bouncing = true;
        // }
        // else if (robot.readBS_Q2()) { // Turn CW 135 degrees if moving forward, CCW 45 degrees if moving backward
        //   Serial.println("Bounce Q2!");
        //   if (forward) {
        //     robot.setSpeedLeft(turn_velocity);
        //     robot.setSpeedRight(-turn_velocity);
        //     bounce_duration = turn_135_time;
        //   }
        //   else {
        //     robot.setSpeedLeft(-turn_velocity);
        //     robot.setSpeedRight(turn_velocity);
        //     bounce_duration = turn_45_time;
        //   }
        //   bounce_start_time = millis();
        //   bouncing = true;
        // }
        // else if (robot.readBS_Q3()) { // Turn CW 45 degrees if moving forward, CCW 135 degrees if moving backward
        //   Serial.println("Bounce Q3!");
        //   if (forward) {
        //     robot.setSpeedLeft(turn_velocity);
        //     robot.setSpeedRight(-turn_velocity);
        //     bounce_duration = turn_45_time;
        //   }
        //   else {
        //     robot.setSpeedLeft(-turn_velocity);
        //     robot.setSpeedRight(turn_velocity);
        //     bounce_duration = turn_135_time;
        //   }
        //   bounce_start_time = millis();
        //   bouncing = true;
        // }
        // else if (robot.readBS_Q4()) { // Turn CCW 45 degrees if moving forward, CW 135 degrees if moving backward
        //   Serial.println("Bounce Q4!");
        //   if (forward) {
        //     robot.setSpeedLeft(-turn_velocity);
        //     robot.setSpeedRight(turn_velocity);
        //     bounce_duration = turn_45_time;
        //   }
        //   else {
        //     robot.setSpeedLeft(turn_velocity);
        //     robot.setSpeedRight(-turn_velocity);
        //     bounce_duration = turn_135_time;
        //   }
        //   bounce_start_time = millis();
        //   bouncing = true;
        // }

// bool forward = true;                 // True if moving forward, false if moving backward
// float turn_135_time = PI*SPACING*135.0/360.0/turn_velocity; // Time to turn 135 degrees
// float turn_45_time = PI*SPACING*45.0/360.0/turn_velocity; // Time to turn 45 degrees