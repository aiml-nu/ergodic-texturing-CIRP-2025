from bleak import BleakScanner, BleakClient
import asyncio
import data

class bluetooth(object):
    def __init__(self,
                 positions_queue: asyncio.Queue,
                 robot_queue: asyncio.Queue,
                 command_queue: asyncio.Queue,
                 stop_event: asyncio.Event,
                 timeout: float
                 ):
        self.devices = {} # Connected devices
        self.clients = {} # Connected clients
        self.name_prefix = "NSFFM_" # Beginning part of robot names, will be followed by a number
        self.timeout = timeout

        # All asyncio stuff
        self.positions_queue = positions_queue
        self.robot_queue = robot_queue
        self.command_queue = command_queue
        self.stop_event = stop_event

        self.positions = data.data_positions()
        self.robot = data.data_robot()
        self.command = data.data_command()

        self.send_positions_index = 0

    async def scan(self):
        devices = await BleakScanner.discover(timeout=self.timeout) 
        self.devices = {}
        for device in devices:
            if self.name_prefix in str(device):
                number_str = str(device).split(self.name_prefix)[1]
                self.devices[number_str] = device
                print("Found client number " + number_str + " which is device ", device)

    async def connect(self):
        def disconnection_handler(client):
            print("Disconnected from:   " + str(client))

        async def robot_noti_handler(characteristic, 
                                     noti_data,
                                     ):
            robot_dict = self.robot.bytesToDict(noti_data) 
            await self.robot_queue.put(robot_dict)

        self.clients = {}
        for number_str in self.devices.keys():
            device = self.devices[number_str]
            print("Attempting to connect to " + number_str)
            client = BleakClient(device, winrt=dict(use_cached_services=False), disconnected_callback=disconnection_handler)
            await client.connect()
            print("Connected to client number " + number_str)
            await client.start_notify(self.robot.uuid, callback=robot_noti_handler)
            print("Listening for robot data notifications from client number " + number_str)
            self.clients[number_str] = client

    async def disconnect(self):
        for number_str in self.clients.keys():
            client = self.clients[number_str]
            print("Disconnecting from client " + number_str)
            await client.disconnect()

    # Grab robot data from the queue and send it to whatever robot is next in line
    async def send_positions(self):
        if not self.positions_queue.empty():
            positions_dict = await self.positions_queue.get()
            client_numbers = list(self.clients.keys())
            client_number = client_numbers[self.send_positions_index]
            client = self.clients[client_number]
            self.send_positions_index = (self.send_positions_index + 1) % len(self.clients.keys())
            await client.write_gatt_char(self.positions.uuid, self.positions.dictToBytes(positions_dict), response=False)

    # Create a task to run this loop
    async def send_positions_loop(self,
                                  min_time: float = 0.02
                                  ):
        while not self.stop_event.is_set():
            await asyncio.gather(asyncio.sleep(min_time),self.send_positions())
        await self.disconnect()

    # Grab command data from the queue and send it to the correct robot
    async def send_command(self):
        if not self.command_queue.empty():
            command_dict = await self.command_queue.get()
            number_str = str(command_dict["NUM"])
            if number_str in self.clients.keys():
                client = self.clients[number_str]
                await client.write_gatt_char(self.command.uuid, self.command.dictToBytes(command_dict), response=False)

    # Create a task to run this loop
    async def send_command_loop(self,
                                min_time: float = 0.02
                                ):
        while not self.stop_event.is_set():
            await asyncio.gather(asyncio.sleep(min_time),self.send_command())

    # This function runs all the updates needed.
    async def update(self,
                     min_time: float = 0.02):
        send_positions_task = asyncio.create_task(self.send_positions_loop(min_time=min_time))
        send_command_task = asyncio.create_task(self.send_command_loop(min_time=min_time))
        await send_positions_task
        await send_command_task
