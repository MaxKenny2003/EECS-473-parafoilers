import serial.tools.list_ports
ports = list(serial.tools.list_ports.comports())
print("Found", len(ports), "ports")
for p in ports:
    print(repr(p))
    print("  device:", p.device)
    print("  name:  ", p.name)
    print("  hwid:  ", p.hwid)
    print("  vid:pid:", (p.vid, p.pid))
    print("  description:", p.description)
    print()