import XInput
from copy import copy
import data
import asyncio

class controller(object):
    def __init__(self,
                 command_active_queue: asyncio.Queue,
                 command_history_queue: asyncio.Queue,
                 stop_event: asyncio.Event
                 ):
        self.command_active_queue = command_active_queue
        self.command_history_queue = command_history_queue
        self.stop_event = stop_event

        self.selection = 0
        self.commands = {str(self.selection): data.data_command()} # Store state in dictionary
        self.command = self.commands[str(self.selection)]
        self.command.data["NUM"] = self.selection
        self.possible_modes = [0, 1, 2] 
        self.minmode = min(self.possible_modes)
        self.maxmode = max(self.possible_modes)
        
    async def update_controller(self):
        events = XInput.get_events()
        for event in events:
            if event.type == XInput.EVENT_BUTTON_PRESSED:
                if event.button == "START": # Pressing the start button should increment the selected robot
                    self.selection = self.selection + 1
                    if not str(self.selection) in self.commands.keys():
                        self.commands[str(self.selection - 1)] = self.command # Store the old command
                        self.command = data.data_command() # Get the new command
                        self.command.data["NUM"] = self.selection # Set the number of this command
                    else:
                        self.commands[str(self.selection - 1)] = self.command # Store the old command
                        self.command = self.commands[str(self.selection)] # Get the new command
                    print("Incremented selected robot to {}!".format(self.selection))

                if event.button == "BACK": # Pressing the back button should decrement the selected robot
                    self.selection = self.selection - 1
                    if not str(self.selection) in self.commands.keys():
                        self.commands[str(self.selection)] = data.data_command()
                        self.command = data.data_command() # Get the new command
                        self.command.data["NUM"] = self.selection # Set the number of this command
                    else:
                        self.commands[str(self.selection + 1)] = self.command # Store the old command
                        self.command = self.commands[str(self.selection)] # Get the new command
                    print("Decremented selected robot to {}!".format(self.selection))

                if event.button == "DPAD_UP": # "DPAD_UP" should increment the mode
                    if self.command.data["MODE"] < self.maxmode:
                        self.command.data["MODE"] = self.command.data["MODE"] + 1
                        print("Incremented mode on robot {} to {}!".format(self.selection,self.command.data["MODE"]))

                if event.button == "DPAD_DOWN": # "DPAD_DOWN" should decrement the mode
                    if self.command.data["MODE"] > self.minmode:
                        self.command.data["MODE"] = self.command.data["MODE"] - 1
                        print("Decremented mode on robot {} to {}!".format(self.selection,self.command.data["MODE"]))

                if event.button == "B": # "B" should toggle oscillation
                    if self.command.data["OSCILLATION"] == True:
                        self.command.data["OSCILLATION"] = False
                    else:
                        self.command.data["OSCILLATION"] = True
                    print("Toggled oscillation on robot {} to {}!".format(self.selection,self.command.data["OSCILLATION"]))

                if event.button == "X": # "X" should toggle the accessory
                    if self.command.data["ACCESSORY"] == True:
                        self.command.data["ACCESSORY"] = False
                    else:
                        self.command.data["ACCESSORY"] = True
                    print("Toggled acccessory on robot {} to {}!".format(self.selection,self.command.data["ACCESSORY"]))
                    
                if event.button == "Y": # "Y" should toggle the reset flag
                    if self.command.data["RESET"] == True:
                        self.command.data["RESET"] = False
                    else:
                        self.command.data["RESET"] = True
                    print("Toggled reset on robot {} to {}!".format(self.selection,self.command.data["RESET"]))

                if event.button == "A": # "A" should toggle autonomous
                    if self.command.data["AUTONOMOUS"] == True:
                        self.command.data["AUTONOMOUS"] = False
                    else:
                        self.command.data["AUTONOMOUS"] = True
                    print("Toggled autonomous on robot {} to {}!".format(self.selection,self.command.data["AUTONOMOUS"]))
                
            if event.type == XInput.EVENT_STICK_MOVED:
                if event.stick == XInput.LEFT:
                    self.command.data["LEFT"] = event.y
                if event.stick == XInput.RIGHT:
                    self.command.data["RIGHT"] = event.y

        # Put the new command data into the queues
        if self.command_active_queue.empty():
            await self.command_active_queue.put(self.command.data)
        if self.command_history_queue.empty():
            await self.command_history_queue.put(self.command.data)

    async def update(self,
                     min_time: float = 0.02
                     ):
        while not self.stop_event.is_set():
            await asyncio.gather(asyncio.sleep(min_time),self.update_controller())
    