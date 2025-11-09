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
from PIL import ImageTk, Image
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

        # Telemetry Values 
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

        # Controls Values
        self.gpsLatitudeVar = tk.DoubleVar()
        self.gpsLongitudeVar = tk.DoubleVar()
        self.adjustAltitudeVar = tk.DoubleVar()
        self.absoluteAltitudeVar = tk.DoubleVar() # Display the Distance between the Structure and the Ground (as opposed to Sea Level)

        # Constant to adjust Altitude in order to gain Absolute Altitude
        # North Campus: 280 Meters above Sealevel
        self.adjustAltitudeVar.set(280.0)
        # Control Mode Radiobutton Labels
        self.modes = ["Auton", "Manual"]
        # Create a shared control variable for both Radiobuttons
        self.modeVar = tk.StringVar(value="Auton")
        self.manualFrameDrawn = False;


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
        top.grid(column=0, row=0);

        # Port Selector Group
        ttk.Label(top, text="Port:").grid(column=0, row=0);
        self.port_cb = ttk.Combobox(top, width=8, values=self._scan_ports())
        self.port_cb.grid(column=1, row=0, padx=4)
        self.port_cb.set(self.port_cb['values'][0] if self.port_cb['values'] else "")

        ttk.Label(top, text="Baud:").grid(column=2, row=0);
        self.baud_cb = ttk.Combobox(top, width=8, values=[9600, 19200, 38400, 57600, 115200])
        self.baud_cb.grid(column=3, row=0, padx=4)
        self.baud_cb.set(DEFAULT_BAUD)

        self.scan_btn = ttk.Button(top, text="Scan Ports", command=self._do_scan)
        self.scan_btn.grid(column=1, row=1, padx=4)

        self.connect_btn = ttk.Button(top, text="Connect", command=self.connect)
        self.connect_btn.grid(column=2, row=1, padx=4)

        self.disconnect_btn = ttk.Button(top, text="Disconnect", command=self.disconnect, state="disabled")
        self.disconnect_btn.grid(column=3, row=1, padx=4)

        # Left Frame: telemetry data + control settings (Auton/Manual, etc)
        self.left = ttk.Frame(self)
        self.left.grid(column=0, row=1)

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
        ttk.Label(self.telemetryGyroFrame, text=' rad/s^2').grid(column=2, row=0)
        ttk.Label(self.telemetryGyroFrame, text=' rad/s^2').grid(column=2, row=1)
        ttk.Label(self.telemetryGyroFrame, text=' rad/s^2').grid(column=2, row=2)
        # Gyroscope Labels: Display the text variable labels
        self.telemetryGyroRollVarLabel = ttk.Label(self.telemetryGyroFrame, textvariable=self.telemetryGyroRollVar).grid(column=1, row=0)
        self.telemetryGyroPitchVarLabel = ttk.Label(self.telemetryGyroFrame, textvariable=self.telemetryGyroPitchVar).grid(column=1, row=1)
        self.telemetryGyroYawVarLabel = ttk.Label(self.telemetryGyroFrame, textvariable=self.telemetryGyroYawVar).grid(column=1, row=2)

        # GPS (Telemetry) Frame
        self.telemetryGPSFrame = ttk.LabelFrame(self.telemetryFrame, text = "GPS")
        self.telemetryGPSFrame.grid(column=0, row=1, ipady=6, sticky="w,e")
        # GPS Labels: Non-variable labels
        ttk.Label(self.telemetryGPSFrame, text='Latitude: ').grid(column=0, row=0)
        ttk.Label(self.telemetryGPSFrame, text='Longitude: ').grid(column=0, row=1)
        ttk.Label(self.telemetryGPSFrame, text='Altitude: ').grid(column=0, row=2)
        ttk.Label(self.telemetryGPSFrame, text='Absolute Altitude: ').grid(column=0, row=3)
        ttk.Label(self.telemetryGPSFrame, text='°').grid(column=2, row=0)
        ttk.Label(self.telemetryGPSFrame, text='°').grid(column=2, row=1)
        ttk.Label(self.telemetryGPSFrame, text='m').grid(column=2, row=2)
        ttk.Label(self.telemetryGPSFrame, text='m').grid(column=2, row=3)
        # GPS Labels: Display the text variable label
        self.telemetryGPSLatVarLabel = ttk.Label(self.telemetryGPSFrame, textvariable=self.telemetryGPSLatVar).grid(column=1, row=0)
        self.telemetryGPSLonVarLabel = ttk.Label(self.telemetryGPSFrame, textvariable=self.telemetryGPSLonVar).grid(column=1, row=1)
        self.telemetryGPSAltVarLabel = ttk.Label(self.telemetryGPSFrame, textvariable=self.telemetryGPSAltVar).grid(column=1, row=2)
        self.telemetryGPSAbsoluteAltVarLabel = ttk.Label(self.telemetryGPSFrame, textvariable=self.absoluteAltitudeVar).grid(column=1, row=3)
        # self.telemetryGPSFrame.config()


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
        middle.grid(column=1, row=1)

        console_frame = ttk.LabelFrame(middle, text="Console")
        console_frame.grid(column=0, row=0)
        self.console = scrolledtext.ScrolledText(console_frame, height=12, state="disabled", wrap="none")
        self.console.pack(fill="both", expand=True, padx=4, pady=4)
  
        # Console Commands (E-Stop, Send Coordinates)
        self.consoleCommandsFrame = ttk.LabelFrame(middle, text="Commands")
        self.consoleCommandsFrame.grid(column=0, row=1)
        # Command button to send the laptop's GPS coordinates
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

        # Adjust Altitude Constant (To make sure that GUI shows proper elevation)
        self.adjustAltitudeLabel = ttk.Label(self.consoleCommandsFrame, text="Subtract Altitude by this value in Meters")
        self.adjustAltitudeLabel.grid(column=0, row=3)
        self.adjustAltitudeEntry = ttk.Entry(self.consoleCommandsFrame, textvariable=self.adjustAltitudeVar)
        self.adjustAltitudeEntry.grid(column=1, row=3)
        # Deployment Button (Add relevant metrics to the side)
        # Todo: Ask Max if there should be some kind of condition before deploying (like only deploy if we're above 100 feet)
        self.deploymentButton = ttk.Button(self.consoleCommandsFrame, text='Deploy Payload', command=self._deployPayloadCmd)
        self.deploymentButton.grid(column=0, row=4)


        #***************************************************#
        #Right frame: Logo, deployment / control metrics, etc#
        #***************************************************#   
        # Right Frame Definition
        right = ttk.Frame(self)
        right.grid(row=1, column=3)

        # Logo for GUI:
        self.imgLogo = ImageTk.PhotoImage(Image.open('shieldSmallNoBG.png'))
        self.logoLabel = ttk.Label(right, image=self.imgLogo).grid(column=0, row=0)
        
        # Controls Frame: Current GPS Location and Heading, as well as distance between
        self.controlsFrame = ttk.LabelFrame(right, text="Controls Frame")
        self.controlsFrame.grid(column=0, row=1)

        # Controls Labels: Non-variable labels
        ttk.Label(self.controlsFrame, text='Current Location: ').grid(column=0, row=2)
        ttk.Label(self.controlsFrame, text='Current Destination: ').grid(column=0, row=3)

        self.controlsLocationLat = ttk.Label(self.controlsFrame, textvariable=self.telemetryGPSLatVar).grid(column=1, row=2)
        self.controlsLocationLon = ttk.Label(self.controlsFrame, textvariable=self.telemetryGPSLonVar).grid(column=2, row=2)

        self.controlsDestLat = ttk.Label(self.controlsFrame, textvariable=self.gpsLatitudeVar).grid(column=1, row=3)
        self.controlsDestLon = ttk.Label(self.controlsFrame, textvariable=self.gpsLongitudeVar).grid(column=2, row=3)

        # RX Data Button (Temporary until RX is on officially looped)
        # todo: Make RX run in an official loop
        self.rxDataButton = ttk.Button(self.consoleCommandsFrame, text = 'Receive Data from TX', command=self._rxSensorData)
        self.rxDataButton.grid(row=2, column=0)

        # LASTLY, perform any neccessy keybinds for general command panel:
        self.emergencyStopButton.bind_all("<space>", self._emergyStopEventCmd)
        # middle.bind('<space>', lambda e: self.emergencyStopButton.invoke())
        # self.
        # root.bind('<Return>', lambda e: action.invoke())

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

    #################################################
    # Left-Middle Frame Functions: Telemetry, Modes #
    #################################################

    # Function to switch to manual mode, redraw control buttons, etc
    # These have been written here to provide scope of upper function to those below
    def toggleModeCheck(self):
        # Check to see if we are in the right condition
        # Case 1: We want to draw the Manual Frame and are in the right conditions
        if (self.manualFrameDrawn == False and self.modeVar.get() == "Manual"):
            # Draw Control Frame
            self.manualFrame = ttk.LabelFrame(self.left, text="Manual Controls")
            self.manualFrame.grid(column=0, row=1)
            # Draw Left and Right buttons
            self.turnLeftButton = ttk.Button(self.manualFrame, text="Turn Left", command=self._manualTurnLeft)
            self.turnLeftButton.grid(column=0, row = 0)
            self.turnRightButton = ttk.Button(self.manualFrame, text="Turn Right", command=self._manualTurnRight)
            self.turnRightButton.grid(column=1, row=0)
            self.manualFrameDrawn = True
            # Bind the relevant functions for controlling the parafoil to the buttons
            self.turnLeftButton.bind_all("<Left>", self._manualTurnLeftEvent)
            self.turnRightButton.bind_all("<Right>", self._manualTurnRightEvent)
            # todo: Fix this keybind to make manual control possible
            # Send a Packet to set the System to Manual Mode
            self._setToManual()
            # Send a Log of the Toggle to Console
            self._log("Toggled to Manual Mode")
            
        # Case 2: We want to destory the Frame  (and potentially draw the Auton frame) and are in the right conditions
        elif (self.manualFrameDrawn == True and self.modeVar.get() == "Auton"):
            # Unbind the relevant functions for controlling the parafoil
            self.turnLeftButton.unbind_all("<Left>")
            self.turnRightButton.unbind_all("<Right>")
            # Destroy the Control Frame from the Left GUI:
            self.manualFrame.destroy()
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
    
    def _manualTurnLeftEvent(self, event=None):
        data = b'L'+ bytes(8)
        self.serial_port.write(data)
        self._log("→ Manual LEFT command sent (via keybind)")

    def _manualTurnRight(self):
        data = b'R' + bytes(8)
        self.serial_port.write(data)
        self._log("→ Manual RIGHT command sent")

    def _manualTurnRightEvent(self, event=None):
        data = b'R' + bytes(8)
        self.serial_port.write(data)
        self._log("→ Manual RIGHT command sent (via keybind)")
        

    # def _manualTurnLeft(self, event=None):
    # # your action
    # pass


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

    def _emergyStopEventCmd(self, event=None):
        data = b'S' + bytes(8)
        self.serial_port.write(data)
        self._log("→ Manual STOP command sent (via keybind)")
        self._log("E-Stoped the Parafoil.")
   
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
        rxDataGPSLat = struct.unpack('f', rxData[0:4])
        rxDataGPSLon = struct.unpack('f', rxData[4:8])
        rxDataGPSAlt = struct.unpack('f', rxData[8:12])
        rxDataAccelX = struct.unpack('f', rxData[12:16])
        rxDataAccelY = struct.unpack('f', rxData[16:20])
        rxDataAccelZ = struct.unpack('f', rxData[20:24])
        rxDataGyroX  = struct.unpack('f', rxData[24:28])
        rxDataGyroY  = struct.unpack('f', rxData[28:32])
        rxDataGyroZ  = struct.unpack('f', rxData[32:36])
        rxDataEulerI = struct.unpack('f', rxData[36:40])
        rxDataEulerJ = struct.unpack('f', rxData[40:44])
        rxDataEulerK = struct.unpack('f', rxData[44:48])
        
        # rxDataStr = rxData.decode('utf-8').strip()
        # Grab specfic data from the sensors for each metric
        # For now, just print out data that is being sent to the console:
        self._log(f"Received Sensor Data: {rxData}")
        # self._log(f"Received Sensor Data as Floats: {rxDataFloat}")
        # self._log(f"Received Sensor Data as Floats: {rxDataFloat2}")
        self.telemetryAccelXVar.set(rxDataAccelX)
        self.telemetryAccelYVar.set(rxDataAccelY)
        self.telemetryAccelZVar.set(rxDataAccelZ)
        self.telemetryGyroRollVar.set(rxDataGyroX)
        self.telemetryGyroYawVar.set(rxDataGyroY)
        self.telemetryGyroPitchVar.set(rxDataGyroZ)
        self.telemetryGPSLatVar.set(rxDataGPSLat)
        self.telemetryGPSLonVar.set(rxDataGPSLon)
        self.telemetryGPSAltVar.set(rxDataGPSAlt)
        self.telemetryEulerIVar.set(rxDataEulerI)
        self.telemetryEulerKVar.set(rxDataEulerJ)
        self.telemetryEulerJVar.set(rxDataEulerK)

    def _deployPayloadCmd(self):
        self._log("Deploying Payload...")
        # self._log(f"Drop Height") #Todo: change this to relflect proper height later
        # Todo: Double check if this is the right char
        data = b'P' + bytes(8)
        self.serial_port.write(data)

    def _periodic_ui_update(self):

        # Run GUI Checks
        self.toggleModeCheck()
        # If we are currently connected, then run the RX Sensor Data Function
        # if(self.serial_port.is_open):
        #     self._rxSensorData()
        # Recalculate self.absoluteAltitudeVar
        self.absoluteAltitudeVar = self.telemetryGPSAltVar.get() - self.adjustAltitudeVar.get();
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


