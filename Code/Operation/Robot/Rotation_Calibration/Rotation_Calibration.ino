#include "../../Robot/Helpers/Robot.cpp"

float voltage_increment = 0.049; // Voltage increment each loop in terms of 0 to 1
float voltage;

Robot robot;

void setup() {
  #ifdef DEBUG
  Serial.println("Beginning robot startup.");
  #endif
  robot.setup();
  #ifdef DEBUG
  Serial.println("Finished robot startup.");
  #endif
}

void loop() {
  voltage = 0.0;
  while (voltage < 1.0) {
    Serial.print("Setting voltage to ");
    Serial.println(voltage);
    // Counterclockwise rotation (theta always increasing)
    // robot.setVoltageLeft(-voltage);
    // robot.setVoltageRight(voltage);
    robot.setVoltageLeft(0.0);
    robot.setVoltageRight(0.0);
    delay(20000); // keep this voltage for 10s
    voltage = voltage + voltage_increment;
  }
}
