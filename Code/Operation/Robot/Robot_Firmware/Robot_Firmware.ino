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

// #define DEBUG

#define ROBOT_ID 4

#define EPSILON 0.01

String robot_name = "NSFFM_" + String(ROBOT_ID);

// Timing stuff
unsigned long start_time = 0;            // Time in millis when the setup ended
unsigned long current_time = 0;          // Time in millis at beginning of current main loop
unsigned long last_send_time = 0;        // Time in millis when data was last shared with the PC
unsigned long last_receive_time = 10000; // Time in millis when data was last received from the PC
unsigned long last_control_time = 0;     // Time in millis when the controls were last updated
unsigned long bounce_start_time = 0;     // Time in millis when a bounce was started
float control_dt = 0.0;                  // Time in seconds since last control update

// Counters
int bounce_count = 0;                 // Number of bounces

// Bounce related numbers
float bounce_duration = 0;                                  // Duration of a bounce
float turn_velocity = 75.0;                                 // Wheel velocity to use when turning during a bounce
float bounce_const = 1000.0*PI*S_N*L/360.0/turn_velocity; // Constant used to speed up calculations
long random_direction = 0;                                  // Random direction from +90 to -90 from opposite the current direction of motion

// Robot behavior
bool bouncing = false;    // Should be true whenever we are actively "bouncing" off of another robot
bool autonomous = false;  // If true, robot will work autonomously 
bool oscillation = false; // If true, tool will oscillate
bool accessory = false;   // If true, accessory will be active
bool reseting = false;    // True whenever we are in the midst of reseting the ck, phik, and time
char mode = 0;            
float left = 0.0;
float right = 0.0;

// Data structures
data_positions data_positions_obj;
data_robot data_robot_obj;
data_command data_command_obj;

// Important objects
ErgodicControl controller;
Robot robot;
RobotBLE ble(robot_name, &data_positions_obj, &data_robot_obj, &data_command_obj);

void setup() {
  Serial.begin(921600);

  data_robot_obj.num = ROBOT_ID;

  #ifdef DEBUG
  Serial.println("Beginning robot startup.");
  #endif
  robot.setup();
  #ifdef DEBUG
  Serial.println("Finished robot startup.");
  #endif

  #ifdef DEBUG
  Serial.println("Beginning controller startup.");
  #endif
  controller.Startup();
  #ifdef DEBUG
  Serial.println("Finished controller startup.");
  #endif

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

void check_bounce() {
  // forward = robot_data.v_left + robot_data.v_right > 0 ? true : false; // Get our direction of motion
  if ((robot.readBS_Q1() || robot.readBS_Q2() || robot.readBS_Q3() || robot.readBS_Q4()) && !bouncing) { // If any bumper activates while not already bouncing
  // if ((robot.readBS_Q1() || robot.readBS_Q2() || robot.readBS_Q4())) { // If any bumper activates while not already bouncing   ROBOT 1 HAS BROKEN Q3
    Serial.println("Bounce!");
    random_direction = random(-90,90);
    if (random_direction < 0) {
      bounce_duration = bounce_const*(180+random_direction);
      robot.setSpeedLeft(-turn_velocity);
      robot.setSpeedRight(turn_velocity);
    }
    else {
      bounce_duration = bounce_const*(180-random_direction);
      robot.setSpeedLeft(turn_velocity);
      robot.setSpeedRight(-turn_velocity);
    }
    bounce_start_time = millis();
    bouncing = true;
    bounce_count = bounce_count + 1;
  }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MAIN LOOP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void loop() {
  current_time = millis();

  if (oscillation) { 
    robot.setOscillation(1.0); 
  }
  else {
    if (robot.readTS()) {
      robot.setOscillation(0.0);
    }
  }

  // Sending data to PC
  if (current_time - last_send_time > SEND_DELAY) {
    last_send_time = current_time;
    data_robot_obj.pd_val = robot.readPD();
    data_robot_obj.metric = controller.GetErgodicMetric(); // Only calculating intermittently since it might be expensive
    data_robot_obj.bumps = bounce_count;
    ble.send_robot();
  }

  // Updating state if there is an available measurement
  if (ble.check_update_positions()) {
    for (char tag=0; tag<MAX_TAGS; tag++) {
      int id = data_positions_obj.get_id(tag);
      if (id == ROBOT_ID) {
        float pos_x = data_positions_obj.get_pos_x(tag)/L + 0.5;
        float pos_y = data_positions_obj.get_pos_y(tag)/L + 0.5;
        float theta = data_positions_obj.get_theta(tag);
        data_robot_obj.pos_x = pos_x;
        data_robot_obj.pos_y = pos_y;
        data_robot_obj.theta = theta;
      }
    }
  }

// Updating robot behavior if there is an available command
if (ble.check_update_command()) {
  autonomous = data_command_obj.autonomous;
  oscillation = data_command_obj.oscillation;
  accessory = data_command_obj.accessory;
  if (reseting == false && data_command_obj.reset == true) {
    #ifdef DEBUG
    Serial.println("Starting Reset!");
    #endif
    reseting = true;
    controller.Reset();
    bounce_count = 0;
  }
  else if (reseting == true && data_command_obj.reset == false) {
    #ifdef DEBUG
    Serial.println("Stopping Reset!");
    #endif
    reseting = false;
  }
  mode = data_command_obj.mode;
  left = data_command_obj.left;
  right = data_command_obj.right;
}

  // Main update 
  if (current_time - last_control_time > SIM_DT*1000) 
  {
    control_dt = 0.001*(float)(current_time - last_control_time);
    last_control_time = current_time;
    #ifdef DEBUG
    Serial.print("Starting control loop after ");
    Serial.print(control_dt);
    Serial.println(" seconds.");
    #endif

    if (!autonomous) { // We are in RC mode
      robot.setVoltageLeft(left);
      robot.setVoltageRight(right);
    }
    else { 
      if (!bouncing) {
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

        // --- Set the motor velocities --- // 
        robot.setSpeedLeft(data_robot_obj.v_left*L);
        robot.setSpeedRight(data_robot_obj.v_right*L);

        // --- Update the result distribution differently depending on the current mode --- //
        if (mode == 0) { // Regular ergodic control
          controller.UpdateResultDistribution(data_robot_obj.pos_x,
                                              data_robot_obj.pos_y,
                                              data_robot_obj.theta,
                                              data_robot_obj.time);
        }
        else if (mode == 1) { // Decentralized ergodic control
          for (char tag=0; tag<MAX_TAGS; tag++) {
            int id = data_positions_obj.get_id(tag);
            if (id != 0) {
              float pos_x = data_positions_obj.get_pos_x(tag)/L + 0.5;
              float pos_y = data_positions_obj.get_pos_y(tag)/L + 0.5;
              float theta = data_positions_obj.get_theta(tag);
              controller.UpdateResultDistribution(pos_x,
                                                  pos_y,
                                                  theta,
                                                  data_robot_obj.time);
            }
          }
        }

        // --- Check whether we have hit another robot; if so, start bouncing --- //
        check_bounce();
      }
      else {
        if (millis() - bounce_start_time > bounce_duration) { // We are done bouncing after a certain time
          bouncing = false;
        }
      }
    }
  }
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