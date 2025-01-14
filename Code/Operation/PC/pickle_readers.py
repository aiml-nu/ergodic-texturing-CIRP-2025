import pickle

class reader(object):
    def __init__(self,
                 pickle_path: str):
        file = open(pickle_path,"rb")
        self.message_sequence = pickle.load(file)

class robot_reader(reader):
    keylist = ['QT','PD_VAL','V_LEFT','V_RIGHT','POS_X','POS_Y','THETA','TIME','METRIC','BUMPS']
    def read(self):
        robot_history = {}
        for message in self.message_sequence:
            num = message['NUM']
            if num in robot_history.keys():
                robot_history[num].append({key:message[key] for key in self.keylist})
            else:
                robot_history[num] = []
        robot_arrays = {}
        for key in robot_history.keys():
            robot_arrays[key] = {}
            for subkey in self.keylist:
                robot_arrays[key][subkey] = []
                for entry in robot_history[key]:
                    robot_arrays[key][subkey].append(entry[subkey])
        return robot_arrays

class positions_reader(reader):
    keylist = ['QT','ID','POS_X','POS_Y','THETA']
    def read(self):
        robot_history = {}
        for message in self.message_sequence:
            for i in range(5):
                num = message['ID_'+str(i)]
                if num in robot_history.keys():
                    entry = {'QT':message['QT'],
                             'ID':num,
                             'POS_X':message['POS_X_'+str(i)],
                             'POS_Y':message['POS_Y_'+str(i)],
                             'THETA':message['THETA_'+str(i)]}
                    robot_history[num].append(entry)
                elif num != 0:
                    robot_history[num] = []
        robot_arrays = {}
        for key in robot_history.keys():
            robot_arrays[key] = {}
            for subkey in self.keylist:
                robot_arrays[key][subkey] = []
                for entry in robot_history[key]:
                    robot_arrays[key][subkey].append(entry[subkey])
        return robot_arrays