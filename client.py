import bleak

class Client:
    def __init__(self):
        self.device: bleak.BleakClient = None
        self.connected: bool = False
    
    async def connect_device(self, device) -> None:
        self.device = bleak.BleakClient(device)
        self.connected = await self.device.connect()
        self.services = self.device.services
    
    async def disconnect_device(self)->None:
        self.device.disconnect()

    @classmethod
    async def scan(cls)->dict:
        return await bleak.BleakScanner.discover(return_adv=True)
    
    @classmethod
    async def scan_device(cls, device_name)->tuple:
        devices = await bleak.BleakScanner.discover(return_adv=True)
        for device, adv in devices.values():
            if(device.name == device_name):
                return device
        return None
    
    async def writeToChar(self, char_uuid, value):
        try:
            await self.device.write_gatt_char(char_uuid, value)
        except Exception as e:
            print("Exception: ", e)
    
    async def servAndChar(self):
        return self.device.services
    #default notification_handler, pass your async funtion as notification_handler while calling start_notify()
    @classmethod
    async def notification_handler(self, char_uuid, data: bytearray)->None:
        print("Data: ", data.decode("utf-8"))
    
    async def start_notification(self, char_uuid, handler = None):
        if(not self.device.is_connected):raise TimeoutError
        if(handler):    await self.device.start_notify(char_uuid, handler)
        else:   await self.device.start_notify(char_uuid, Client.notification_handler)

    
    async def stop_notification(self, char_uuid):
        await self.device.stop_notify(char_uuid)
    
"""_summary_
Notes:
    1.  After enabling notification remember to keep it alive using an async looping function.
        Eg: while(client.is_connected): await asyncio.sleep(1)
        This will keep the program running. 
"""