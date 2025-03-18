from client import Client
import calculation
import body
import time
import struct
import asyncio
import bleak
import asyncio
import pyautogui

pyautogui.PAUSE = 0.0
pyautogui.FAILSAFE = False

gravity:list = [0, 0, 0]
location:list = [0, 0, 0]
dy_loc:list = [0, 0, 0]

accel_data:list = [0, 0, 0]
gyro_data:list = [0, 0, 0]
magnet_data:list = [0, 0, 0]

kalman_roll = body.KalmanFilter()
kalman_pitch = body.KalmanFilter()
kalman_yaw = body.KalmanFilter()
dt = 0.01
float()
kalman_filters = [kalman_roll, kalman_pitch, kalman_yaw]

async def setData(Character, data: bytearray):
    data = getData(data)
    if(Character.uuid == "00000011-0000-1000-8000-00805f9b34fb"): gyro_data[0] = round(data, 1)
    if(Character.uuid == "00000012-0000-1000-8000-00805f9b34fb"): gyro_data[1] = round(data, 1)
    if(Character.uuid == "00000013-0000-1000-8000-00805f9b34fb"): gyro_data[2] = round(data, 1)
    if(Character.uuid == "00000021-0000-1000-8000-00805f9b34fb"): accel_data[0] = round(data, 1)
    if(Character.uuid == "00000022-0000-1000-8000-00805f9b34fb"): accel_data[1] = round(data, 1)
    if(Character.uuid == "00000023-0000-1000-8000-00805f9b34fb"): accel_data[2] = round(data, 1)
    if(Character.uuid == "00000031-0000-1000-8000-00805f9b34fb"): magnet_data[0] = round(data, 1)
    if(Character.uuid == "00000032-0000-1000-8000-00805f9b34fb"): magnet_data[1] = round(data, 1)
    if(Character.uuid == "00000033-0000-1000-8000-00805f9b34fb"): magnet_data[2] = round(data, 1)
# async def printdata(Character, data: bytearray):
#     print(Character.uuid)
#     print(type(Character))

def getData(byte_array, byte_order='little'):
    return struct.unpack('f', byte_array)[0]

async def cursorMove(x, y)->None:
    # pyautogui.mouseDown(button="left")
    pyautogui.move(x, y)
    # pyautogui.mouseUp(button="left")

async def cursor_y(CHARACTER, data: bytearray)->None:
    # pyautogui.mouseDown(button="left")
    accel = getData(byte_array=data)
    print("Y: ",accel)
    pixels_y = calculation.compute_displacement(accel)
    pyautogui.move(0, pixels_y)
    # pyautogui.mouseUp(button="left")

def setLocationData(ax, ay, az):
    displacement_x = calculation.compute_displacement(ax, scale_factor=1)
    displacement_y = calculation.compute_displacement(ay, scale_factor=1)
    displacement_z = calculation.compute_displacement(az, scale_factor=1)
    
    dy_loc[0] = displacement_x
    dy_loc[1] = displacement_y
    dy_loc[2] = displacement_z
    
    location[0] = location[0] + displacement_x
    location[1] = location[1] + displacement_y
    location[2] = location[2] + displacement_z
    # print(displacement_x, "\t", displacement_y, "\t", displacement_z)

async def main():
    CHAR_GX = "00000011-0000-1000-8000-00805f9b34fb"
    CHAR_GY = "00000012-0000-1000-8000-00805f9b34fb"
    CHAR_GZ = "00000013-0000-1000-8000-00805f9b34fb"
    CHAR_AX = "00000021-0000-1000-8000-00805f9b34fb"
    CHAR_AY = "00000022-0000-1000-8000-00805f9b34fb"
    CHAR_AZ = "00000023-0000-1000-8000-00805f9b34fb"
    CHAR_MX = "00000031-0000-1000-8000-00805f9b34fb"
    CHAR_MY = "00000032-0000-1000-8000-00805f9b34fb"
    CHAR_MZ = "00000033-0000-1000-8000-00805f9b34fb"
    CHAR_FLAG = "00002a57-0000-1000-8000-00805f9b34fb"

    print("Scanning..")
    client = Client()
    await Client.scan()
    server_add = await Client.scan_device("Nano33BLE_SingleValue")
    # server_add = "06:70:DB:B3:CD:E8"

    if(not server_add):
        print("No Device Found")
        return

    # print("Server Add: ", server_add)
    # print("-------------------------------------------------------------------------------------------")

    try:
        await client.connect_device(server_add)
    except bleak.exc.BleakError as e:
        print(f"Connection failed: {e}")
    except TimeoutError as e:
        print(f"Timeout error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

    if(client.connected):print("Connected")
    else:
        print("Connection Failed.")
        return

    print("-------------------------------------------------------------------------------------------")
    await client.servAndChar()
    print("-------------------------------------------------------------------------------------------")

    try:
        await client.start_notification(CHAR_GX, handler=setData)
        await client.start_notification(CHAR_GY, handler=setData)
        await client.start_notification(CHAR_GZ, handler=setData)
        await client.start_notification(CHAR_AX, handler=setData)
        await client.start_notification(CHAR_AY, handler=setData)
        await client.start_notification(CHAR_AZ, handler=setData)
        await client.start_notification(CHAR_MX, handler=setData)
        await client.start_notification(CHAR_MY, handler=setData)
        await client.start_notification(CHAR_MZ, handler=setData)
    except Exception as e:print(e)
    print("Conformation Sent.")

    await client.writeToChar(CHAR_FLAG, b'\x01')
    while(client.connected):
        pitch, roll, yaw = body.calculate_orientation(accel_data, gyro_data, dt, kalman_filters)
        gravity = calculation.getGrav(90-pitch, 90-roll, yaw)
        setLocationData((accel_data[0] - gravity[0]), (accel_data[1] - gravity[1]), (accel_data[2] - gravity[2]))
        #scale is set here internally
        await cursorMove(int(dy_loc[0] * 150), int(dy_loc[1] * 150))
        await asyncio.sleep(0.01)
        print(pitch, "\t", roll, "\t", yaw)
        # print(pitch, "\t", roll, "\t", yaw, "\t", gravity)
        # print(location, "\t", dy_loc)
        print("===================================================================================================================================")
        # print("Gravity: ", gravity)
        # print("Accelld: ", accel_data)
        # print("Location:\t\t",location)
        # print("PIXELS:\t\t",dy_loc)

    await client.stop_notification(CHAR_GX)
    await client.stop_notification(CHAR_GY)
    await client.stop_notification(CHAR_GY)
    await client.stop_notification(CHAR_AX)
    await client.stop_notification(CHAR_AY)
    await client.stop_notification(CHAR_AY)
    await client.stop_notification(CHAR_MX)
    await client.stop_notification(CHAR_MY)
    await client.stop_notification(CHAR_MY)
    await client.disconnect_device()
    print("Disconnected")


if __name__ == "__main__":
    #try:
        asyncio.run(main())
    # except Exception as e:
    #     print(e)
    # finally:
    #     pyautogui.mouseUp(button="left")