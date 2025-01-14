import struct

class data(object):
    def __init__(self):
        self.data = {}
        self.types = {}
        self.bytes = {}
        self.uuid = ""
    
    # Convert a dictionary to a bytearray
    def dictToBytes(self,
                    input_dict):
        output_bytes = b''
        for type,val in zip(self.types,input_dict.values()):
            if type == 'B':
                if val < 0:
                    val = 0
                elif val > 255:
                    val = 255
            output_bytes = output_bytes + struct.pack(type,val)
        return bytearray(output_bytes)

    # Convert a bytearray to a dictionary
    def bytesToDict(self, 
                    input_bytes):
        output_dict = self.data.copy()
        index = 0
        for type,key,bytes in zip(self.types,self.data.keys(),self.bytes):
            output_dict[key] = struct.unpack(type,input_bytes[index:index+bytes])[0]
            index = index + bytes
        return output_dict

class data_positions(data):
    def __init__(self):
        self.data = {"ID_0"    : 0,   # Tag 0 ID, if the value is 0 then it is not present
                     "POS_X_0" : 0.0, # Tag 0 position in x
                     "POS_Y_0" : 0.0, # Tag 0 position in y
                     "THETA_0" : 0.0, # Tag 0 angle
                     "ID_1"    : 0,   
                     "POS_X_1" : 0.0,   
                     "POS_Y_1" : 0.0,   
                     "THETA_1" : 0.0,   
                     "ID_2"    : 0,
                     "POS_X_2" : 0.0,   
                     "POS_Y_2" : 0.0,   
                     "THETA_2" : 0.0,  
                     "ID_3"    : 0,   
                     "POS_X_3" : 0.0,   
                     "POS_Y_3" : 0.0,   
                     "THETA_3" : 0.0,  
                     "ID_4"    : 0,   
                     "POS_X_4" : 0.0,   
                     "POS_Y_4" : 0.0,   
                     "THETA_4" : 0.0}
        self.types = ["B","f","f","f",
                      "B","f","f","f",
                      "B","f","f","f",
                      "B","f","f","f",
                      "B","f","f","f"]
        self.bytes = [1,4,4,4,
                      1,4,4,4,
                      1,4,4,4,
                      1,4,4,4,
                      1,4,4,4]
        self.uuid = "4fea0cb7-f189-4d1c-85e4-7ae6e490e182"
        self.max = 5

    def fill(self,
             measurements):
        new_data = self.data.copy()
        count = 0
        for num, measurement in measurements.items():
            new_data["ID_" + str(count)] = int(num)
            new_data["POS_X_" + str(count)] = measurement["X"]
            new_data["POS_Y_" + str(count)] = measurement["Y"]
            new_data["THETA_" + str(count)] = measurement["THETA"]
            count = count + 1
            if count == self.max:
                break
        return new_data
    
class data_robot(data):
    def __init__(self):
        self.data = {"NUM"       : 0,   # Number of the robot generating this data
                     "PD_VAL"    : 0.0, # Value of photodetector from 0 to 1
                     "V_LEFT"    : 0.0, # Left wheel velocity, [mm/s]
                     "V_RIGHT"   : 0.0, # Right wheel velocity, [mm/s]
                     "POS_X"     : 0.0, # The robot's measured X position, [mm]
                     "POS_Y"     : 0.0, # The robot's measured Y position, [mm]
                     "THETA"     : 0.0, # The robot's measured angle, [rad]
                     "TIME"      : 0.0, # Time when the data was generated, [s]
                     "METRIC"    : 0.0, # Value of the ergodic metric
                     "BUMPS"     : 0}   # Number of bumps the robot has experienced
        self.types = ["B",
                      "f","f","f","f","f","f","f","f",
                      "i"]
        self.bytes = [1,
                      4,4,4,4,4,4,4,4,
                      4]
        self.uuid = "441fbc49-5922-4fc9-b6e7-aa97fb05c6f1"
        
class data_command(data):
    def __init__(self):
        self.data = {"NUM"         : 0,     # Number of the robot being commanded
                     "AUTONOMOUS"  : False, # If True, the robot runs without human control
                     "OSCILLATION" : False, # If True, the robot tool will oscillate vertically
                     "ACCESSORY"   : False, # If True, the accessory will be on
                     "RESET"       : False, # If True, the robot will reset ck and recalculate phik
                     "MODE"        : 0,     # Mode code: 0 = independent ergodic, 1 = decentralized ergodic, 2 = stigmergic
                     "LEFT"        : 0.0,   # Motor power for left wheel, 0 to 1
                     "RIGHT"       : 0.0}   # Motor power for right wheel, 0 to 1
        self.types = ["B",
                      "?","?","?","?",
                      "B",
                      "f","f"]
        self.bytes = [1,
                      1,1,1,1,
                      1,
                      4,4]
        self.uuid = "61bc568c-b996-4d57-a30a-5aa720837488"