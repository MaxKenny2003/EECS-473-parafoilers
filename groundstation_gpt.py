import serial
import struct

ser = serial.Serial(
    port='COM6',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

def validate_coordinates(lat, lon):
    """Validate latitude and longitude ranges."""
    try:
        lat_f = float(lat)
        lon_f = float(lon)
        
        if not (-90 <= lat_f <= 90):
            print(f"Error: Latitude must be between -90 and 90")
            return None, None
        
        if not (-180 <= lon_f <= 180):
            print(f"Error: Longitude must be between -180 and 180")
            return None, None
        
        return lat_f, lon_f
    except ValueError:
        print("Error: Invalid number format")
        return None, None

def coords_to_fixed_point(lat, lon, scale=10000000):
    """
    Convert decimal coordinates to fixed-point integers.
    
    Args:
        lat: Latitude in decimal degrees
        lon: Longitude in decimal degrees
        scale: Multiplier (10^7 gives ~1cm precision)
    
    Returns:
        tuple: (lat_fixed, lon_fixed) as integers
    """
    lat_fixed = int(lat * scale)
    lon_fixed = int(lon * scale)
    return lat_fixed, lon_fixed

if ser.is_open:
    print(f"{ser.name} serial port is opened")
    print("\nPARAFOIL GROUND STATION")
    print("\nCommands:")
    print("  coord  - Send target GPS coordinates (auto mode)")
    print("  L      - Manual control: Turn LEFT")
    print("  R      - Manual control: Turn RIGHT")
    print("  S      - Manual control: STOP servo")
    print("  A      - Switch to AUTO (GPS navigation) mode")
    print("  M      - Switch to MANUAL OVERRIDE mode")
    print("  exit   - Close serial port and quit")
    print("=" * 31 + "\n")
else:
    print("Error: port not opened")
    exit(1)

try:
    while True:
        cmd = input("Enter CMD: ")

        if cmd == "exit":
            print("Closing serial port...")
            ser.close()
            print("Serial port closed")
            break
        
        elif cmd == "coord":
            latitude = input("Enter latitude (-90 to 90): ")
            longitude = input("Enter longitude (-180 to 180): ")

            # Validate input
            lat_f, lon_f = validate_coordinates(latitude, longitude)
            if lat_f is None or lon_f is None:
                continue

            # Convert to fixed-point
            SCALE = 10000000  # 10^7 for ~1cm precision
            lat_fixed, lon_fixed = coords_to_fixed_point(lat_f, lon_f, SCALE)
            
            # Check bounds (important!)
            if not (-900000000 <= lat_fixed <= 900000000):
                print("Error: Latitude out of fixed-point range")
                continue
            if not (-1800000000 <= lon_fixed <= 1800000000):
                print("Error: Longitude out of fixed-point range")
                continue
            
            # Pack as little-endian signed 32-bit integers
            # '<i' = little-endian signed int (4 bytes)
            lat_bytes = struct.pack('>i', lat_fixed)
            lon_bytes = struct.pack('>i', lon_fixed)
            
            # Verify
            print(f"Original: Lat={lat_f}, Lon={lon_f}")
            print(f"Fixed-point: Lat={lat_fixed}, Lon={lon_fixed}")
            print(f"Lat bytes: {lat_bytes.hex()}")
            print(f"Lon bytes: {lon_bytes.hex()}")
            print(f"Reconstructed: Lat={lat_fixed/SCALE}, Lon={lon_fixed/SCALE}")
            
            # Create message: 'C' (1 byte) + lat (4 bytes) + lon (4 bytes)
            data = b'C' + lat_bytes + lon_bytes
            
            print(f"Sending {len(data)} bytes: {data.hex()}")
            ser.write(data)
        
        elif cmd == "L":
            data = b'L' + bytes(8)
            ser.write(data)
            print("→ Manual LEFT command sent\n")
            
        elif cmd == "R":
            data = b'R' + bytes(8)
            ser.write(data)
            print("→ Manual RIGHT command sent\n")
            
        elif cmd == "S":
            data = b'S' + bytes(8)
            ser.write(data)
            print("→ Manual STOP command sent\n")
            
        elif cmd == "A":
            data = b'A' + bytes(8)
            ser.write(data)
            print("→ AUTO (GPS navigation) mode command sent\n")
            
        elif cmd == "M":
            data = b'M' + bytes(8)
            ser.write(data)
            print("→ MANUAL OVERRIDE mode command sent\n")
            
        else:
            print(f"Unknown command: '{cmd}'")
            print("Type 'exit' for help or 'exit' to quit\n")

except KeyboardInterrupt:
    print("\nInterrupted by user")
finally:
    if ser.is_open:
        ser.close()
        print("Serial port closed")