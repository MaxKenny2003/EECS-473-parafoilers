#!/usr/bin/env python3
"""
xbee_dashboard.py
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import time
from collections import deque
# import matplotlib
# matplotlib.use("TkAgg")
# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# from matplotlib.figure import Figure

DEFAULT_BAUD = 9600
PLOT_POINTS = 100
GUI_UPDATE_PERIOD = 50


# Include groundstation python code to make GUI Functional
import struct


class XBeeDashboard(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Groundstation XBee Dashboard")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.serial_port = None
        self.stop_event = threading.Event()
        self.rx_thread = None

        # Telemetry values 
        self.telemetryAccelXVar = tk.DoubleVar()
        self.telemetryAccelYVar = tk.DoubleVar()
        self.telemetryAccelZVar = tk.DoubleVar()
        self.telemetryGyroRollVar  = tk.DoubleVar()
        self.telemetryGyroPitchVar = tk.DoubleVar()
        self.telemetryGyroYawVar   = tk.DoubleVar()
        self.telemetryGPSLatVar = tk.DoubleVar()
        self.telemetryGPSLonVar = tk.DoubleVar()
        self.telemetryGPSAltVar = tk.DoubleVar()
        self.telemetryEulerIVar = tk.DoubleVar() 
        self.telemetryEulerKVar = tk.DoubleVar()
        self.telemetryEulerJVar = tk.DoubleVar()

        # Control Mode Radiobutton Labels
        self.modes = ["Auton", "Manual"]
        # Create a shared control variable for both Radiobuttons
        self.modeVar = tk.StringVar(value="Auton")
        self.manualFrameDrawn = False;

        # Create Entry Variables for the Send GPS Coords Button
        self.gpsLatitudeVar = tk.DoubleVar()
        self.gpsLongitudeVar = tk.DoubleVar()

        # Build UI and then Disable Widgets until we connect to Serial
        self._build_ui()
        self._init_disable()
        self.after(500, self._periodic_ui_update)

    def _build_ui(self):
        # Main Frame: This will be the top-most level Frame of the GUI
        # (This can be thought of as the "root node" of our tree of frames)

        # main = ttk.Frame(self)

        #***************************************#
        # Top frame: Serial Connection Controls #
        #***************************************#

        top = ttk.Frame(self)
        # top = ttk.Frame(main)
        # top.pack(side="top", fill="x", padx=8, pady=6)
        top.grid(column=0, row=0);

        # Port Selector Group
        ttk.Label(top, text="Port:").grid(column=0, row=0);
        self.port_cb = ttk.Combobox(top, width=8, values=self._scan_ports())
        #self.port_cb.pack(side="left", padx=4)
        self.port_cb.grid(column=1, row=0, padx=4)
        self.port_cb.set(self.port_cb['values'][0] if self.port_cb['values'] else "")

       # ttk.Label(top, text="Baud:").pack(side="left", padx=(10,0))
        ttk.Label(top, text="Baud:").grid(column=2, row=0);
        self.baud_cb = ttk.Combobox(top, width=8, values=[9600, 19200, 38400, 57600, 115200])
        #self.baud_cb.pack(side="left", padx=4)
        self.baud_cb.grid(column=3, row=0, padx=4)
        self.baud_cb.set(DEFAULT_BAUD)

        self.scan_btn = ttk.Button(top, text="Scan Ports", command=self._do_scan)
        # self.scan_btn.pack(side="left", padx=6)
        self.scan_btn.grid(column=1, row=1, padx=4)

        self.connect_btn = ttk.Button(top, text="Connect", command=self.connect)
        # self.connect_btn.pack(side="left", padx=6)
        self.connect_btn.grid(column=2, row=1, padx=4)

        self.disconnect_btn = ttk.Button(top, text="Disconnect", command=self.disconnect, state="disabled")
        # self.disconnect_btn.pack(side="left", padx=2)
        self.disconnect_btn.grid(column=3, row=1, padx=4)


        # Left Frame: telemetry data + control settings (Auton/Manual, etc)
        self.left = ttk.Frame(self)
        self.left.grid(column=0, row=1)
        # left.pack(side="")


        # Reserve the following spots in the grid for each frame:
        # For Dashboard and DashboardTwo, use column = 0, and then rows=0 and 1
        # For Console use column = 1 and rows = 0
        # For Console buttons and commands, make a seperate frame and put it at column = 1, rows = 1
        # For Map Frame, place it at column = 2 and rows = 0

        #**********************************************#
        # Left-Middle frame: telemetry widgets and data#
        #**********************************************#

        # Telemetry Frames
        self.telemetryFrame = ttk.LabelFrame(self.left, text="Telemetry")
        # dash_frame.pack(side="left", fill="y", padx=(0,8))
        self.telemetryFrame.grid(column=0, row=0, ipady=12)

        # Accelerometer (Telemetry) Frame
        self.telemetryAccelFrame = ttk.LabelFrame(self.telemetryFrame, text= "Accelerometer")
        self.telemetryAccelFrame.grid(column=0, row=0, ipady=6)
        # Accelerometer Labels: Make the non-variable labels floating variables, since they won't need to change
        ttk.Label(self.telemetryAccelFrame, text='Acceleration X: ').grid(column=0, row=0)
        ttk.Label(self.telemetryAccelFrame, text='Acceleration Y: ').grid(column=0, row=1)
        ttk.Label(self.telemetryAccelFrame, text='Acceleration Z: ').grid(column=0, row=2)
        ttk.Label(self.telemetryAccelFrame, text=' m/s^2').grid(column=2, row=0)
        ttk.Label(self.telemetryAccelFrame, text=' m/s^2').grid(column=2, row=1)
        ttk.Label(self.telemetryAccelFrame, text=' m/s^2').grid(column=2, row=2)
        # Accelerometer Labels: Display the text variable labels
        self.telemetryAccelXVarLabel = ttk.Label(self.telemetryAccelFrame, textvariable=self.telemetryAccelXVar).grid(column=1, row=0)
        self.telemetryAccelYVarLabel = ttk.Label(self.telemetryAccelFrame, textvariable=self.telemetryAccelYVar).grid(column=1, row=1)
        self.telemetryAccelZVarLabel = ttk.Label(self.telemetryAccelFrame, textvariable=self.telemetryAccelZVar).grid(column=1, row=2)

        # Gyroscope (Telemetry) Frame
        self.telemetryGyroFrame = ttk.LabelFrame(self.telemetryFrame, text= "Gyroscope")
        self.telemetryGyroFrame.grid(column=1, row=0, ipady=6)
        # Gyroscope Labels: Non-variable labels
        ttk.Label(self.telemetryGyroFrame, text='Roll: ').grid(column=0, row=0)
        ttk.Label(self.telemetryGyroFrame, text='Pitch: ').grid(column=0, row=1)
        ttk.Label(self.telemetryGyroFrame, text='Yaw: ').grid(column=0, row=2)
        # todo: not sure if we are measuring in degrees or radians. Ask Max later
        ttk.Label(self.telemetryGyroFrame, text=' rad/s^2').grid(column=2, row=0)
        ttk.Label(self.telemetryGyroFrame, text=' rad/s^2').grid(column=2, row=1)
        ttk.Label(self.telemetryGyroFrame, text=' rad/s^2').grid(column=2, row=2)
        # Gyroscope Labels: Display the text variable labels
        self.telemetryGyroRollVarLabel = ttk.Label(self.telemetryGyroFrame, textvariable=self.telemetryGyroRollVar).grid(column=1, row=0)
        self.telemetryGyroPitchVarLabel = ttk.Label(self.telemetryGyroFrame, textvariable=self.telemetryGyroPitchVar).grid(column=1, row=1)
        self.telemetryGyroYawVarLabel = ttk.Label(self.telemetryGyroFrame, textvariable=self.telemetryGyroYawVar).grid(column=1, row=2)

        # GPS (Telemetry) Frame
        self.telemetryGPSFrame = ttk.LabelFrame(self.telemetryFrame, text = "GPS")
        self.telemetryGPSFrame.grid(column=0, row=1, ipady=6)
        # GPS Labels: Non-variable labels
        ttk.Label(self.telemetryGPSFrame, text='Latitude: ').grid(column=0, row=0)
        ttk.Label(self.telemetryGPSFrame, text='Longitude: ').grid(column=0, row=1)
        ttk.Label(self.telemetryGPSFrame, text='Altitude: ').grid(column=0, row=2)
        ttk.Label(self.telemetryGPSFrame, text='°').grid(column=2, row=0)
        ttk.Label(self.telemetryGPSFrame, text='°').grid(column=2, row=1)
        ttk.Label(self.telemetryGPSFrame, text='m').grid(column=2, row=2)
        # GPS Labels: Display the text variable label
        self.telemetryGPSLatVarLabel = ttk.Label(self.telemetryGPSFrame, textvariable=self.telemetryGPSLatVar).grid(column=1, row=0)
        self.telemetryGPSLonVarLabel = ttk.Label(self.telemetryGPSFrame, textvariable=self.telemetryGPSLonVar).grid(column=1, row=1)
        self.telemetryGPSAltVarLabel = ttk.Label(self.telemetryGPSFrame, textvariable=self.telemetryGPSAltVar).grid(column=1, row=2)

        # Euler Angles (Telemetry) Frame
        self.telemetryEulerFrame = ttk.LabelFrame(self.telemetryFrame, text = "Euler Angles")
        self.telemetryEulerFrame.grid(column=1, row=1, ipady=6)
        # Euler Angles Labels: Non-variable labels
        ttk.Label(self.telemetryEulerFrame, text= 'I Angle: ').grid(column=0, row=0)
        ttk.Label(self.telemetryEulerFrame, text= 'J Angle: ').grid(column=0, row=1)
        ttk.Label(self.telemetryEulerFrame, text= 'K Angle: ').grid(column=0, row=2)
        ttk.Label(self.telemetryEulerFrame, text= ' rad').grid(column=2, row=0)
        ttk.Label(self.telemetryEulerFrame, text= ' rad').grid(column=2, row=1)
        ttk.Label(self.telemetryEulerFrame, text= ' rad').grid(column=2, row=2)
        # Euler Angles Labels: Display the text variable label
        self.telemetryEulerIVarLabel = ttk.Label(self.telemetryEulerFrame, textvariable=self.telemetryEulerIVar).grid(column=1, row=0)
        self.telemetryEulerJVarLabel = ttk.Label(self.telemetryEulerFrame, textvariable=self.telemetryEulerJVar).grid(column=1, row=1)
        self.telemetryEulerKVarLabel = ttk.Label(self.telemetryEulerFrame, textvariable=self.telemetryEulerKVar).grid(column=1, row=2)
        # "Mode" Frame
        self.modesFrame = ttk.LabelFrame(self.left, text="Modes")
        self.modesFrame.grid(column=5, row=0)
        self.auton = ttk.Radiobutton(self.modesFrame, text=self.modes[0], variable=self.modeVar, value="Auton")
        self.auton.grid(row=1, column=2, sticky="w", padx=6, pady=6)
        self.manual = ttk.Radiobutton(self.modesFrame, text=self.modes[1], variable=self.modeVar, value="Manual")
        self.manual.grid(row=2, column=2, sticky="w", padx=6, pady=6)

        #***************************************************#
        #Middle frame: dashboard + console + button commands#
        #***************************************************#

        middle = ttk.Frame(self)
        # middle.pack(side="top", fill="both", expand=True, padx=8, pady=6)
        middle.grid(column=1, row=1)

        console_frame = ttk.LabelFrame(middle, text="Console")
        # console_frame.pack(side="top", fill="both", expand=True)
        console_frame.grid(column=0, row=0)
        self.console = scrolledtext.ScrolledText(console_frame, height=12, state="disabled", wrap="none")
        self.console.pack(fill="both", expand=True, padx=4, pady=4)
  
        # Console Commands (E-Stop, Send Coordinates)
        self.consoleCommandsFrame = ttk.LabelFrame(middle, text="Commands")
        self.consoleCommandsFrame.grid(column=0, row=1)
        # Command button to send the laptop's GPS coordinates
        # button = ttk.Button(parent, text='Okay', command=submitForm)
        self.emergencyStopButton = ttk.Button(self.consoleCommandsFrame, text="Emergency Stop / Deadfall", command=self._emergencyStopCmd)
        self.emergencyStopButton.grid(column=0, row=0)


        # Send GPS Coordinates (Button and Entries)
        self.sendGPSButton = ttk.Button(self.consoleCommandsFrame, text='Send Current GPS Coordinates', command=self._sendGPSCmd)
        self.sendGPSButton.grid(column=0, row=1)
        sendGPSLatitudeLabel = ttk.Label(self.consoleCommandsFrame, text='Latitude: ')
        sendGPSLatitudeLabel.grid(column=1, row=1)
        self.sendGPSLatitudeEntry = ttk.Entry(self.consoleCommandsFrame, textvariable=self.gpsLatitudeVar)
        self.sendGPSLatitudeEntry.grid(column=2, row=1)
        sendGPSLongitudeLabel = ttk.Label(self.consoleCommandsFrame, text='Longitude: ')
        sendGPSLongitudeLabel.grid(column=3, row=1)
        self.sendGPSLongitudeEntry = ttk.Entry(self.consoleCommandsFrame, textvariable=self.gpsLongitudeVar)
        self.sendGPSLongitudeEntry.grid(column=4, row=1)        
        # Right: Map and other Data
        right = ttk.Frame(self)
        # right.pack(side="left", fill="both", expand=True)
        right.grid(row=1, column=3)

        # RX Data Button (Temporary until RX is on officially looped)
        self.rxDataButton = ttk.Button(self.consoleCommandsFrame, text = 'Receive Data from TX', command=self._rxSensorData)
        self.rxDataButton.grid(row=2, column=0)

    def _init_disable(self):
        # Disable several buttons and widgets upon start up
        # Disable Left Frame Buttons
        # Disable Example:  self.connect_btn.config(state="disabled")
        self.manual.config(state="disabled")
        self.auton.config(state="disabled")
        # Disable Middle Frame Buttons
        self.emergencyStopButton.config(state="disabled")
        self.sendGPSButton.config(state="disabled")
        self.sendGPSLatitudeEntry.config(state="disabled")
        self.sendGPSLongitudeEntry.config(state="disabled")
        
    def _init_enable(self):
        # Enable several buttons and widegets upon start up
        # Enable Left Frame Buttons
        self.manual.config(state="normal")
        self.auton.config(state="normal")
        # Enable Middle Frame Buttons
        self.emergencyStopButton.config(state="normal")
        self.sendGPSButton.config(state="normal")
        self.sendGPSLatitudeEntry.config(state="normal")
        self.sendGPSLongitudeEntry.config(state="normal")
        # Lastly, Enable the Auton Mode by default
        self._log("Enabling Auton Mode by Default")
        self._setToAuto()
        
    
        
    ######################################################
    # Top Frame Functions: Serial, Connection, and others#
    ######################################################

    def _scan_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        return ports

    def _do_scan(self):
        ports = self._scan_ports()
        self.port_cb['values'] = ports
        if ports:
            self.port_cb.set(ports[0])
        self._log("Scanned ports: " + ", ".join(ports))

    def connect(self):
        port = self.port_cb.get()
        try:
            baud = int(self.baud_cb.get())
        except:
            baud = DEFAULT_BAUD

        if not port:
            messagebox.showwarning("Port required", "Please choose a serial port.")
            return

        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baud,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1   
            )
        except Exception as e:
            messagebox.showerror("Open port failed", f"Could not open {port}: {e}")
            return
        if(self.serial_port.is_open):
            # Only enable buttons if the port we connected to is open
            self.connect_btn.config(state="disabled")
            self.disconnect_btn.config(state="normal")
            self.scan_btn.config(state="disabled")
            self._init_enable()
            self.serialConnectLog()
        else:
            self._log("Error: port not opened")
            print("Error: port not opened")
            exit(1)
        # self.stop_event.clear()
        # self.rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
        # self.rx_thread.start()

    def disconnect(self):
        self.stop_event.set()
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
            self.rx_thread = None
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.serial_port = None
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        self.scan_btn.config(state="normal")
        self._init_disable()
        self._log("Disconnected")

    # Comment out the Thread Functions for now. 
    # See if it will be useful later
    # def _rx_worker(self):
    #     buff = bytearray()
    #     while not self.stop_event.is_set():
    #         try:
    #             if self.serial_port is None:
    #                 break
    #             data = self.serial_port.read(128)
    #             if data:
    #                 buff.extend(data)
    #                 # handle lines
    #                 while b'\n' in buff:
    #                     idx = buff.index(b'\n')
    #                     line = buff[:idx+1].decode(errors='replace').strip()
    #                     buff = buff[idx+1:]
    #                     self._handle_line(line)
    #             else:
    #                 time.sleep(0.01)
    #         except Exception as e:
    #             self._log(f"Serial read error: {e}")
    #             time.sleep(0.5)

    # def _handle_line(self, line):
    #     # Display raw line
    #     self._log("RX: " + line)

    #     # Expect telemetry as KEY=VALUE;KEY=VALUE;...\n
    #     try:
    #         parts = line.strip().split(';')
    #         changed = False
    #         for p in parts:
    #             if '=' in p:
    #                 k,v = p.split('=',1)
    #                 k=k.strip().upper()
    #                 v=v.strip()
    #                 if k in self.telemetry:
    #                     self.telemetry[k] = v
    #                     changed = True
    #                     if k == "TEMP":
    #                         try:
    #                             self.temp_history.append(float(v))
    #                         except:
    #                             pass
    #         if changed:
    #             self._update_dashboard_widgets()
    #     except Exception as e:
    #         # not telemetry or parse error — ignore for dashboard
    #         pass



    #################################################
    # Left-Middle Frame Functions: Telemetry, Modes #
    #################################################

    # Function to switch to manual mode, redraw control buttons, etc
    # These have been written here to provide scope of upper function to those below
    def toggleModeCheck(self):
        # Check to see if we are in the right condition
        # Case 1: We want to draw the Manual Frame and are in the right conditions
        if (self.manualFrameDrawn == False and self.modeVar.get() == "Manual"):
            # self.drawControlFrame()
            # telemetryFrame = ttk.LabelFrame(left, text="Telemetry")
            # Draw Control Frame
            self.controlFrame = ttk.LabelFrame(self.left, text="Manual Controls")
            self.controlFrame.grid(column=0, row=1)
            # Draw Left and Right buttons
            self.turnLeftButton = ttk.Button(self.controlFrame, text="Turn Left", command=self._manualTurnLeft)
            self.turnLeftButton.grid(column=0, row = 0)
            self.turnRightButton = ttk.Button(self.controlFrame, text="Turn Right", command=self._manualTurnRight)
            self.turnRightButton.grid(column=1, row=0)
            self.manualFrameDrawn = True
            # Send a Packet to set the System to Manual Mode
            self._setToManual()
            # Send a Log of the Toggle to Console
            self._log("Toggled to Manual Mode")
            
        # Case 2: We want to destory the Frame  (and potentially draw the Auton frame) and are in the right conditions
        # Todo: ask team what metrics / inputs should be needed for this section
        elif (self.manualFrameDrawn == True and self.modeVar.get() == "Auton"):
            # Destroy the Control Frame from the Left GUI:
            self.controlFrame.destroy()
            self.manualFrameDrawn = False
            # Send a Packet to set the System to Manual Mode
            self._setToAuto()
            # Send a Log of the Toggle to Console
            self._log("Toggled to Autonomous Mode")

    def serialConnectLog(self):
        self._log(f"{self.serial_port.name} serial port is opened")
        self._log(f"Connected to {self.serial_port.port} @ {self.serial_port.baudrate}")
        self._log("\nPARAFOIL GROUND STATION")
        self._log("\nCommands:")
        self._log("  coord  - Send target GPS coordinates using the button in 'Commands' Frame (auto mode)")
        self._log("  Latitude Range: [-90, 90], Longitude Range: [-180, 180]")
        self._log("  L      - Manual control: Turn LEFT")
        self._log("  R      - Manual control: Turn RIGHT")
        self._log("  S      - Manual control: STOP servo")
        self._log("  A      - Switch to AUTO (GPS navigation) mode")
        self._log("  M      - Switch to MANUAL OVERRIDE mode")
        self._log("  exit   - Close serial port and quit")
        self._log("=" * 31 + "\n")
    
    def _setToAuto(self):
        data = b'A' + bytes(8)
        self.serial_port.write(data)
        self._log("→ AUTO (GPS navigation) mode command sent")
    
    def _setToManual(self):
        data = b'M' + bytes(8)
        self.serial_port.write(data)
        self._log("→ MANUAL OVERRIDE mode command sent")

    def _manualTurnLeft(self):
        data = b'L'+ bytes(8)
        self.serial_port.write(data)
        self._log("→ Manual LEFT command sent")
    
    def _manualTurnRight(self):
        data = b'R' + bytes(8)
        self.serial_port.write(data)
        self._log("→ Manual RIGHT command sent")


    ##############################################################
    # Middle Frame Functions: Console, Console Commands, and etc.#
    ##############################################################

    def _log(self, text):
        # append to console thread-safely using after
        ts = time.strftime("%H:%M:%S")
        self.console.configure(state="normal")
        self.console.insert("end", f"[{ts}] {text}\n")
        self.console.see("end")
        self.console.configure(state="disabled")

    # def _update_dashboard_widgets(self):
        # self.lbl_TEMP.config(text=str(self.telemetry["TEMP"]))
        # self.lbl_HUM.config(text=str(self.telemetry["HUM"]))
        # self.lbl_BAT.config(text=str(self.telemetry["BAT"]))

    def _emergencyStopCmd(self):
        data = b'S' + bytes(8)
        self.serial_port.write(data)
        self._log("→ Manual STOP command sent")
        self._log("E-Stoped the Parafoil.")
        # Todo: Ask team if the GUI should go "unresponsiive" after the function
   
    def consoleValidateCoordinates(self, lat, lon):
        """Validate latitude and longitude ranges."""
        try:
            lat_f = float(lat)
            lon_f = float(lon)
            
            if not (-90 <= lat_f <= 90):
                # print(f"Error: Latitude must be between -90 and 90")
                self._log("Error: Latitude must be between -90 and 90")
                return None, None
            
            if not (-180 <= lon_f <= 180):
                # print(f"Error: Longitude must be between -180 and 180")
                self._log("Error: Longitude must be between -180 and 180")
                return None, None
            
            return lat_f, lon_f
        except ValueError:
            # print("Error: Invalid number format")
            self._log("Error: Invalid number format")
            return None, None
    def consoleCoordsToFixedPoint(self, lat, lon, scale=10000000):
        latFixed = int(lat*scale)
        longFixed = int(lon*scale)
        return latFixed, longFixed

    def _sendGPSCmd(self):
        # Place holder values until entry widgets have been set up
        # Grab Double Values from the GUI entries
        latStr = str(self.gpsLatitudeVar.get())
        longStr = str(self.gpsLongitudeVar.get())
        # Validate Coordinates
        latF, longF = self.consoleValidateCoordinates(latStr, longStr)
        if latF is None or longF is None:
            return
        
        # Convert to fixed-point
        SCALE = 10000000
        latFixed, longFixed = self.consoleCoordsToFixedPoint(latF, longF, SCALE);
        
        # Check bounds for fixed point
        if not (-900000000 <= latFixed <= 900000000):
            self._log("Error: Latitude out of fixed-point range")
            return
        if not (-1800000000 <= longFixed <= 1800000000):
            self._log("Error: Longitude out of fixed-point range")
            return

        # Pack both fixed-point variables as 32-bit integers
        latBytes = struct.pack('>i', latFixed)
        longBytes = struct.pack('>i', longFixed)

        # Print relevant conversion information
        self._log(f"Original: Latitude={latF}, Longitude={longF}")
        self._log(f"Fixed-point: Lat={latFixed}, Lon={longFixed}")
        self._log(f"Lat bytes: {latBytes.hex()}")
        self._log(f"Lon bytes: {longBytes.hex()}")
        self._log(f"Reconstructed: Lat={latFixed/SCALE}, Lon={longFixed/SCALE}")
        
        # Create Packet to send:
        data = b'C' + latBytes + longBytes
        self._log(f"Sending {len(data)} bytes: {data.hex()}")
        self.serial_port.write(data)
        self._log("Rerouted coordinates to " + latStr + "," + longStr)
        # latStr = str(12.111)
        # longStr = str(-10.001)
    def _rxSensorData(self):
        # rxData = self.serial_port.read(64)
        # Alt method to read serial data
        rxData = self.serial_port.readline()
        rxDataFloat = struct.unpack('f', rxData[0:4])
        rxDataFloat2 = struct.unpack('f', rxData[4:8])
        # rxDataStr = rxData.decode('utf-8').strip()
        # Grab specfic data from the sensors for each metric
        # For now, just print out data that is being sent to the console:
        self._log(f"Received Sensor Data: {rxData}")
        self._log(f"Received Sensor Data as Floats: {rxDataFloat}")
        self._log(f"Received Sensor Data as Floats: {rxDataFloat2}")
    def _periodic_ui_update(self):

        # Run GUI Checks
        self.toggleModeCheck()
        # If we are currently connected, then run the RX Sensor Data Function
        # if(self.serial_port.is_open):
        #     self._rxSensorData()
        self.after(GUI_UPDATE_PERIOD, self._periodic_ui_update)

    def _send_text(self):
        text = self.send_entry.get().strip()
        if not text:
            return
        if self.serial_port and self.serial_port.is_open:
            try:
                if not text.endswith("\n"):
                    text = text + "\n"
                self.serial_port.write(text.encode())
                self._log("TX: " + text.strip())
                self.send_entry.delete(0, "end")
            except Exception as e:
                self._log("Send failed: " + str(e))
        else:
            messagebox.showwarning("Not connected", "Open a serial connection first.")

    def on_close(self):
        self.disconnect()
        self.destroy()

if __name__ == "__main__":
    app = XBeeDashboard()
    app.mainloop()


