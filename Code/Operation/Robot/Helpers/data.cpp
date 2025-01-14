#ifndef DATA
#define DATA

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define DATA_POSITIONS_LENGTH 65 // 5*1 + 5*3*4 
#define MAX_TAGS 5 // Maximum number of tags transmitted

class data_positions{
  public:
    u_int8_t *raw_data_ptr; // Pointer to the data straight from BLE
    u_int8_t bytes[DATA_POSITIONS_LENGTH]; // Storage for copied data
  
    data_positions() {}
  
    void update_bytes_from_ptr(u_int8_t *new_raw_data_ptr) {
      raw_data_ptr = new_raw_data_ptr;
      memcpy(bytes, raw_data_ptr, DATA_POSITIONS_LENGTH);
    }

    float get_id(char tag) {
      char id;
      int offset = 13*(int)tag;
      memcpy(&id, &bytes[offset], 1);
      return id;
    }

    float get_pos_x(char tag) {
      float pos_x;
      int offset = 13*(int)tag + 1;
      memcpy(&pos_x, &bytes[offset], 4);
      return pos_x;
    }

    float get_pos_y(char tag) {
      float pos_y;
      int offset = 13*(int)tag + 5;
      memcpy(&pos_y, &bytes[offset], 4);
      return pos_y;
    }

    float get_theta(char tag) {
      float theta;
      int offset = 13*(int)tag + 9;
      memcpy(&theta, &bytes[offset], 4);
      return theta;
    }
};

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define DATA_ROBOT_LENGTH 37 // 1*1 + 8*4 + 1*4 

class data_robot{
  public:
    u_int8_t *raw_data_ptr; // Pointer to the data straight from BLE
    u_int8_t bytes[DATA_ROBOT_LENGTH]; // Storage for copied data

    char  num;     // Number of the robot generating this data
    float pd_val;  // Measurement from the photodiode from 0 to 1
    float v_left;  // Velocity of left wheel
    float v_right; // Velocity of right wheel
    float pos_x;   // Current robot position belief in x
    float pos_y;   // Current robot position belief in y 
    float theta;   // Current robot angle belief in x
    float time;    // Time when this data was generated
    float metric;  // Current value of the ergodic metric
    int   bumps;   // Number of bumps the robot has encountered
  
    data_robot() {
      num = 0;
      pd_val = 0.0;
      v_left = 0.0;
      v_right = 0.0;
      pos_x = 0.0;
      pos_y = 0.0;
      theta = 0.0;
      time = 0.0;
      metric = 0.0;
      bumps = 0;
    }
  
    void update_bytes_from_ptr(u_int8_t *new_raw_data_ptr) {
      raw_data_ptr = new_raw_data_ptr;
      memcpy(bytes, raw_data_ptr, DATA_ROBOT_LENGTH);
    }

    void update_members_from_bytes() {
      memcpy(&num,     &bytes[0],  1);
      memcpy(&pd_val,  &bytes[1],  4);
      memcpy(&v_left,  &bytes[5],  4);
      memcpy(&v_right, &bytes[9],  4);
      memcpy(&pos_x,   &bytes[13], 4);
      memcpy(&pos_y,   &bytes[17], 4);
      memcpy(&theta,   &bytes[21], 4);
      memcpy(&time,    &bytes[25], 4);
      memcpy(&metric,  &bytes[29], 4);
      memcpy(&bumps,   &bytes[33], 4);
    }

    void update_bytes_from_members() {
      memcpy(&bytes[0],  &num,     1);
      memcpy(&bytes[1],  &pd_val,  4);
      memcpy(&bytes[5],  &v_left,  4);
      memcpy(&bytes[9],  &v_right, 4);
      memcpy(&bytes[13], &pos_x,   4);
      memcpy(&bytes[17], &pos_y,   4);
      memcpy(&bytes[21], &theta,   4);
      memcpy(&bytes[25], &time,    4);
      memcpy(&bytes[29], &metric,  4);
      memcpy(&bytes[33], &bumps,   4);
    }
};

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define DATA_COMMAND_LENGTH 14 // 2*1 + 4*1 + 2*4 

class data_command{
  public:
    u_int8_t *raw_data_ptr; // Pointer to the data straight from BLE
    u_int8_t bytes[DATA_COMMAND_LENGTH]; // Storage for copied data

    char  num;         // Number of the robot being commanded
    bool  autonomous;  // If True, calculate behavior using Ergodic Control and bump sensors. If False, use game controller inputs
    bool  oscillation; // If True, tool will oscillate vertically
    bool  accessory;   // If True, accessory will be active
    bool  reset;       // If True, reset ck to zeros and recompute phik
    char  mode;        // Mode code: 0 = independent ergodic, 1 = decentralized ergodic, 2 = stigmergic
    float left;        // Motor voltage for left motor, 0 to 1
    float right;       // Motor voltage for right motor, 0 to 1
  
    data_command() {
      num = 0;
      autonomous = false;
      oscillation = false;
      accessory = false;
      reset = false;
      mode = 0;
      left = 0.0;
      right = 0.0;
    }
  
    void update_bytes_from_ptr(u_int8_t *new_raw_data_ptr) {
      raw_data_ptr = new_raw_data_ptr;
      memcpy(bytes, raw_data_ptr, DATA_COMMAND_LENGTH);
    }

    void update_members_from_bytes() {
      memcpy(&num,         &bytes[0],  1);
      memcpy(&autonomous,  &bytes[1],  1);
      memcpy(&oscillation, &bytes[2],  1);
      memcpy(&accessory,   &bytes[3],  1);
      memcpy(&reset,       &bytes[4],  1);
      memcpy(&mode,        &bytes[5],  1);
      memcpy(&left,        &bytes[6],  4);
      memcpy(&right,       &bytes[10], 4);
    }

    void update_bytes_from_members() {
      memcpy(&bytes[0],  &num,         1);
      memcpy(&bytes[1],  &autonomous,  1);
      memcpy(&bytes[2],  &oscillation, 1);
      memcpy(&bytes[3],  &accessory,   1);
      memcpy(&bytes[4],  &reset,       1);
      memcpy(&bytes[5],  &mode,        1);
      memcpy(&bytes[6],  &left,        4);
      memcpy(&bytes[10], &right,       4);
    }
};

#endif 