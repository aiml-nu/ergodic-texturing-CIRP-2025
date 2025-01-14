// This abstracts away the low-level robot stuff

#ifndef ROBOT
#define ROBOT

// Arduino libraries
#include <Arduino.h>
#include <Wire.h>
// #include <Adafruit_LSM6DSOX.h>
#include <Adafruit_PWMServoDriver.h>

// PINS         
#define BS_1     D6 // Bump sensor 1
#define BS_2     D7 // Bump sensor 2
#define BS_3     D0 // Bump sensor 3
#define BS_4     D2 // Bump sensor 4
#define PD       A1 // Photodetector analog input
#define TS       D3 // Tool top position sensor

// PWM EXPANDER PINS
#define DRV_L_IN1 6 // Left motor IN1
#define DRV_L_IN2 7 // Left motor IN2
#define DRV_R_IN1 0 // Right motor IN1
#define DRV_R_IN2 1 // Right motor IN2
#define OSC_IN1   2 // Oscillation motor IN1   2 for robots (2 thru 5), 3 for robot (1)
#define OSC_IN2   3 // Oscillation motor IN2   3 for robots (2 thru 5), 2 for robot (1)
#define ACS_IN1   4 // Accessory motor IN1
#define ACS_IN2   5 // Accessory motor IN2

// ROBOT PARAMETERS
#define OSCILLATION_PCT 4095 // Level of tool oscillation, between 0 and 4095
#define SEND_DELAY 500       // Time between data sent to PC from Robot in millis
#define MOTOR_MINIMUM 1.0    // Velocity below which the motor should actually be switched off
#define MOTOR_MAXIMUM 75.0   // Maximum velocity the motor should be able to obtain
#define DEADZONE 0.001       // Motor deadzone (out of 1)

// MOTOR FIT PARAMETERS
#define MOTOR_A 0.00001237 // Coefficient on exponential
#define MOTOR_B 0.1439     // Coefficient on voltage within exponential
#define MOTOR_C 0.002291   // Coefficient on linear term
#define MOTOR_D 0.04290    // Constant term

// MATHEMATICAL CONSTANTS
#ifndef PI
#define PI 3.1415
#endif

class Robot {
  public:
    // Adafruit_LSM6DSOX IMU;
    Adafruit_PWMServoDriver PWM;
    // sensors_event_t accel_event; // sensor event for storing accelerometer readings, [m/s^2]
    // sensors_event_t gyro_event;  // sensor event for storing gyroscope readings, [rad/s]
    // sensors_event_t temp_event;  // sensor event for storing temperature readings, [deg C]

  Robot() {
    // IMU = Adafruit_LSM6DSOX();
    PWM = Adafruit_PWMServoDriver();
  }

  // Should be run in the setup() portion of the Arduino code
  void setup() {
    // Turn on serial communications
    Serial.begin(921600);

    // Put the I2C connection into "fast-mode"
    Wire.setClock(400000);

    // // Connect to the IMU
    // if (!IMU.begin_I2C()) {
    //   Serial.println("Could not connect to IMU!");
    // }
    // // Set the IMU ranges and data rates
    // // TODO

    // Connect to the PWM expander
    if (!PWM.begin()) {
      Serial.println("Could not connect to PWM expander!");
    }
    // Set the PWM frequency
    PWM.setPWMFreq(1600);

    // Set all the PWM pins to fully off
    PWM.setPWM(ACS_IN1, 0, 4096);

    // Set other pins correctly
    pinMode(BS_1, INPUT);
    pinMode(BS_2, INPUT);
    pinMode(BS_3, INPUT);
    pinMode(BS_4, INPUT);
    pinMode(PD,   INPUT);
    pinMode(TS,   INPUT);
  }

  // ~~~~~~~~~~ Sensors ~~~~~~~~~~ //

  // // Read from the IMU and store the results
  // void readIMU() {
  //   IMU.getEvent(&accel_event, &gyro_event, &temp_event);
  // }

  float readPD() {
    // Read the photodiodes and return a float from 0 to 1
    return analogRead(PD)/4095.0; // 12-bit ADC outputs from 0 to 4095 integer values
  }

  bool readTS() {
    // Read the top position sensor and return 0 or 1
    return (bool)digitalRead(TS);
  }

  bool readBS_Q1() {
    // Read the bump sensor in the 1st quadrant (robot front right) and return 0 or 1
    return (bool)digitalRead(BS_3);
  }

  bool readBS_Q2() {
    // Read the bump sensor in the 2nd quadrant (robot front left) and return 0 or 1
    return (bool)digitalRead(BS_4);
  }

  bool readBS_Q3() {
    // Read the bump sensor in the 3rd quadrant (robot back left) and return 0 or 1
    return (bool)digitalRead(BS_1);
  }

  bool readBS_Q4() {
    // Read the bump sensor in the 4th quadrant (robot back right) and return 0 or 1
    return (bool)digitalRead(BS_2);
  }

  // ~~~~~~~~~~ Motor Commands ~~~~~~~~~~ //

  float motor_fit(float speed) {
    // Conversion from speed to voltage found experimentally
    return MOTOR_A * pow(2.7183, MOTOR_B * speed) + MOTOR_C * speed + MOTOR_D;
  }

  int speedTo12bit(float speed) {
    // Converts a float speed in [mm/s] to a value between 0 and 4095
    // Uses the fit motor model
    if (speed < MOTOR_MINIMUM)      { return 0; }
    else if (speed < MOTOR_MAXIMUM) { return int(4095*motor_fit(speed)); }
    else                            { return 4095; }
  }

  int voltageTo12bit(float voltage) {
    // Converts a float voltage between 0 and 1 to a value between 0 and 4095
    if (voltage < DEADZONE) { return 0; }
    else if (voltage < 1.0) { return int(4095*voltage); }
    else                    { return 4095; }
  }

  void setMotorVelocity(float velocity, int pin_fwd, int pin_rev) {
    // Set the linear velocity at a wheel motor
    // Expects a value between -MAX_VEL and MAX_VEL
    int magnitude = speedTo12bit(abs(velocity));
    if (velocity < 0) { PWM.setPWM(pin_fwd, 0, magnitude); 
                        PWM.setPWM(pin_rev, 0, 4096); } // fully off
    else              { PWM.setPWM(pin_fwd, 0, 4096); // fully off 
                        PWM.setPWM(pin_rev, 0, magnitude); }  
  }
  
  void setMotorVoltage(float voltage, int pin_fwd, int pin_rev) {
    // Set the linear velocity at a motor
    // Expects a value between -1.0 and 1.0
    int magnitude = voltageTo12bit(abs(voltage));
    if (voltage < 0) { PWM.setPWM(pin_fwd, 0, magnitude); 
                       PWM.setPWM(pin_rev, 0, 4096); } // fully off
    else             { PWM.setPWM(pin_fwd, 0, 4096); // fully off 
                       PWM.setPWM(pin_rev, 0, magnitude); } 
  }

  void setSpeedLeft(float velocity) {
    setMotorVelocity(velocity, DRV_L_IN1, DRV_L_IN2);
  } 

  void setSpeedRight(float velocity) {
    setMotorVelocity(-velocity, DRV_R_IN1, DRV_R_IN2);
  }

  void setVoltageLeft(float voltage) {
    setMotorVoltage(voltage, DRV_L_IN1, DRV_L_IN2);
  } 

  void setVoltageRight(float voltage) {
    setMotorVoltage(-voltage, DRV_R_IN1, DRV_R_IN2);
  } 

  void setOscillation(float voltage) {
    setMotorVoltage(voltage, OSC_IN1, OSC_IN2);
  } 

  void setAccessory(float voltage) {
    setMotorVoltage(-voltage, ACS_IN1, ACS_IN2);
  } 
  
};

#endif