import serial
import time
from fixedpoint import FixedPoint

ser = serial.Serial(
    port = 'COM5',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

if(ser.is_open):
    print(ser.name + ' serial port is opened')
    print("Ground Station Control: ")
    print("CMDS: exit, coord, switch")
else:
    print("Error: port not opened")


while 1:
    cmd = input("Enter CMD: ")

    if(cmd == "exit"):
        print("Serial port closed")
        ser.close()
    elif(cmd == "coord"):
        latitude = input("Enter new coordinate - latitude: ")
        longitude = input("Enter new coordinate - longitude: ")

        lat_bytes = struct.pack('f', float(latitude))
        long_bytes = struct.pack('f', float(longitude))
        #if(len(lat_bytes) > 3):

        data = 'C'.encode("utf-8") + lat_bytes + long_bytes
        ser.write(data)
        print(data.hex())
    else:
        ser.write(str.encode(cmd))


