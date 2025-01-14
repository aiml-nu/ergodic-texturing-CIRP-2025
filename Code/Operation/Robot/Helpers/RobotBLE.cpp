// This contains the Bluetooth Low Energy code

#ifndef ROBOTBLE
#define ROBOTBLE

// Custom data structures
#include "data.cpp"

// BLE Libraries
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// Core Arduino libraries
#include <Arduino.h>

// BLE ID INFORMATION
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define DATA_POSITIONS_UUID "4fea0cb7-f189-4d1c-85e4-7ae6e490e182"
#define DATA_ROBOT_UUID     "441fbc49-5922-4fc9-b6e7-aa97fb05c6f1"
#define DATA_COMMAND_UUID   "61bc568c-b996-4d57-a30a-5aa720837488"

class RobotBLE {
  public:
    bool client_connected;
    bool positions_available;
    bool command_available;
    BLEServer *pServer;
    BLEService *pService;
    BLECharacteristic *pCharacteristic_positions;
    BLECharacteristic *pCharacteristic_robot;
    BLECharacteristic *pCharacteristic_command;
    BLEAdvertising *pAdvertising;
    String robot_name;
    data_positions *data_positions_ptr;
    data_robot *data_robot_ptr;
    data_command *data_command_ptr;

  // Constructor that takes in pointers to data structures
  RobotBLE(String name, 
           data_positions *new_data_positions_ptr,
           data_robot *new_data_robot_ptr,
           data_command *new_data_command_ptr){
    client_connected = false;
    positions_available = false;
    command_available = false;
    robot_name = name;
    data_positions_ptr = new_data_positions_ptr;
    data_robot_ptr = new_data_robot_ptr;
    data_command_ptr = new_data_command_ptr;
  }

  // Special server callback that restarts advertising in case of a disconnection
  class ServerCallback: public BLEServerCallbacks {
    public:
      RobotBLE *ble;
    ServerCallback(RobotBLE *ble) {
      ServerCallback::ble = ble;
    }
    void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) {
      ble->client_connected = true;
      pServer->updateConnParams(param->connect.remote_bda,6,12,0,500);
    }
    void onDisconnect(BLEServer *pServer) {
      pServer->startAdvertising();
      ble->client_connected = false;
    }
  };

  // Special write callback that stores incoming position data
  class DataPositionsCallback: public BLECharacteristicCallbacks {
    public:
      RobotBLE *ble;
    DataPositionsCallback(RobotBLE *ble) {
      DataPositionsCallback::ble = ble;
    }
    void onWrite(BLECharacteristic *pCharacteristic) {
      u_int8_t *recv_data = pCharacteristic->getData();
      size_t recv_length = pCharacteristic->getLength();
      if (recv_length == DATA_POSITIONS_LENGTH) { 
        ble->data_positions_ptr->update_bytes_from_ptr(recv_data);
      }
      ble->positions_available = true;
    }
  };

  class DataCommandCallback: public BLECharacteristicCallbacks {
    public:
      RobotBLE *ble;
    DataCommandCallback(RobotBLE *ble) {
      DataCommandCallback::ble = ble;
    }
    void onWrite(BLECharacteristic *pCharacteristic) {
      u_int8_t *recv_data = pCharacteristic->getData();
      size_t recv_length = pCharacteristic->getLength();
      if (recv_length == DATA_COMMAND_LENGTH) { 
        ble->data_command_ptr->update_bytes_from_ptr(recv_data);
        ble->data_command_ptr->update_members_from_bytes();
      }
      ble->command_available = true;
    }
  };

  // Should be run in the setup() of the main Arduino code
  void setup() {
    BLEDevice::init(robot_name); // Create the BLE device with the given name
    BLEDevice::setMTU(DATA_POSITIONS_LENGTH+10); // Using the positions data length because it is longer
    pServer = BLEDevice::createServer(); // Create the server
    pServer->setCallbacks(new ServerCallback(this)); // Add the necessary callbacks on the server level
    pService = pServer->createService(SERVICE_UUID); // Create the overall service
    pCharacteristic_positions = pService->createCharacteristic(DATA_POSITIONS_UUID, BLECharacteristic::PROPERTY_WRITE);
    pCharacteristic_positions->setCallbacks(new DataPositionsCallback(this)); // Attach the write callback
    pCharacteristic_command = pService->createCharacteristic(DATA_COMMAND_UUID, BLECharacteristic::PROPERTY_WRITE);
    pCharacteristic_command->setCallbacks(new DataCommandCallback(this)); // Attach the write callback
    pCharacteristic_robot = pService->createCharacteristic(DATA_ROBOT_UUID, BLECharacteristic::PROPERTY_READ | 
                                                                            BLECharacteristic::PROPERTY_NOTIFY); // Data transfer from robot to PC
    pCharacteristic_robot->addDescriptor(new BLE2902());
    pService->start(); // Start the service
    pAdvertising = pServer->getAdvertising(); // Get advertising from the server
    pAdvertising->start(); // Start the advertising (broadcasting active BLE signal which can be connected to)
  }

  // Send out the data in the robot data structure
  void send_robot() {
    if (client_connected) {
      data_robot_ptr->update_bytes_from_members();
      u_int8_t *send_data = data_robot_ptr->bytes;
      pCharacteristic_robot->setValue(send_data, DATA_ROBOT_LENGTH);
      pCharacteristic_robot->notify();
    } 
  }

  // Returns true if there is a position update waiting and resets its flag
  bool check_update_positions() {
    if (positions_available) {
      positions_available = false;
      return true;
    }
    else {
      return false;
    }
  }

  // Returns true if there is a command update waiting and resets its flag
  bool check_update_command() {
    if (command_available) {
      command_available = false;
      return true;
    }
    else {
      return false;
    }
  }
};

#endif