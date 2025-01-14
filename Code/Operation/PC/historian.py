import asyncio
import time
import pickle
import numpy as np

class historian(object):
    def __init__(self,
                 history_queue: asyncio.Queue,
                 stop_event: asyncio.Event,
                 printing: bool,
                 filename: str):
        self.history_queue = history_queue # Queue containing data from an outside source
        self.stop_event = stop_event       # Event to halt the historian
        self.history = []                  # The list containing the history constructed from the queue
        self.start_time = time.time()      # Time when the historian was initialized, all data will subtract off this time
        self.printing = printing           # Whether of not we should print data as it comes in
        self.filename = filename

    async def update_history(self):   
        if not self.history_queue.empty():
            data = await self.history_queue.get()            # Retrieve the latest data from the queue
            new_data = {"QT": time.time() - self.start_time} # Create a new dictionary and add the time of reciept as an item
            new_data.update(data)                            # Add the data from the queue onto the new dictionary
            if self.printing:                                # Check whether we should be printing this data
                for key,val in new_data.items():
                    print(key,end=': ')
                    if type(val) == float or type(val) == np.float64:
                        print("{:7.2f}".format(val),end='')
                    else:
                        print(val,end='')
                    print(' | ',end='')
                print('')
            self.history.append(new_data)                    # Add this complete dictionary onto the main history list

    def write_history(self):
        with open(self.filename+".pickle",'wb') as handle:
            pickle.dump(self.history, handle) # Write the history to a pickle
    
    # This function should actually be run as a task and includes a restriction on how often the updates will actually run.
    async def update(self,
                     min_time: float = 0.02
                     ):
        while not self.stop_event.is_set():
            await asyncio.gather(asyncio.sleep(min_time),self.update_history())
        self.write_history()

        
