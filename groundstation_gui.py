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

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

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
        self.telemetryFusedLatVar = tk.DoubleVar()
        self.telemetryFusedLonVar = tk.DoubleVar()
        self.telemetryFusedHeadVar = tk.DoubleVar()
        self.telemetryEkfPosNVar  = tk.DoubleVar()
        self.telemetryEkfPosEVar = tk.DoubleVar()
        self.telemetryEkfVelNVar   = tk.DoubleVar()
        self.telemetryEkfVelEVar = tk.DoubleVar()
        self.telemetryEkfBiasVar = tk.DoubleVar()
        self.telemetryRawGPSLatVar = tk.DoubleVar()
        self.telemetryRawGPSLonVar = tk.DoubleVar()
        self.telemetryRawGPSSpeedVar = tk.DoubleVar()
        self.telemetryRawGPSHeadVar = tk.DoubleVar()
        
        self.rollVar = tk.DoubleVar()
        self.pitchVar = tk.DoubleVar()
        self.yawVar = tk.DoubleVar()
        
        # [15-17] Gyroscope
        self.gyroXVar = tk.DoubleVar()
        self.gyroYVar = tk.DoubleVar()
        self.gyroZVar = tk.DoubleVar()
        
        # [18-20] Accelerometer
        self.accelXVar = tk.DoubleVar()
        self.accelYVar = tk.DoubleVar()
        self.accelZVar = tk.DoubleVar()
        
        # [21-23] Quaternion
        self.quatWVar = tk.DoubleVar()
        self.quatXVar = tk.DoubleVar()
        self.quatYVar = tk.DoubleVar()

        # Control Mode Radiobutton Labels
        self.modes = ["Auton", "Manual"]
        # Create a shared control variable for both Radiobuttons
        self.modeVar = tk.StringVar(value="Auton")
        self.manualFrameDrawn = False

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
        top.grid(column=0, row=0)

        # Port Selector Group
        ttk.Label(top, text="Port:").grid(column=0, row=0);
        self.port_cb = ttk.Combobox(top, width=8, values=self._scan_ports())
        #self.port_cb.pack(side="left", padx=4)
        self.port_cb.grid(column=1, row=0, padx=4)
        self.port_cb.set(self.port_cb['values'][0] if self.port_cb['values'] else "")

       # ttk.Label(top, text="Baud:").pack(side="left", padx=(10,0))
        ttk.Label(top, text="Baud:").grid(column=2, row=0)
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


        # ===========================
        # MAP FRAME (Real-Time 2D Plot)
        # ===========================
        self.mapFrame = ttk.LabelFrame(self, text="2D EKF Map")
        self.mapFrame.grid(column=1, row=1, padx=10, pady=10)

        # Create Figure for map
        self.fig = Figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("East (m)")
        self.ax.set_ylabel("North (m)")
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True)

        # Line for trajectory history
        self.path_east = []
        self.path_north = []
        (self.path_line,) = self.ax.plot([], [], linewidth=2)

        # Arrow for heading
        self.heading_arrow = None

        # Embed figure in Tk
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.mapFrame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill=tk.BOTH, expand=True)


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
        self.telemetryEKFFrame = ttk.LabelFrame(self.telemetryFrame, text= "EKF State (NED)")
        self.telemetryEKFFrame.grid(column=0, row=0, ipady=6)
        ttk.Label(self.telemetryEKFFrame, text='Pos North: ').grid(column=0, row=0)
        ttk.Label(self.telemetryEKFFrame, text='Pos East: ').grid(column=0, row=1)
        ttk.Label(self.telemetryEKFFrame, text='Vel North: ').grid(column=0, row=2)
        ttk.Label(self.telemetryEKFFrame, text='Vel East: ').grid(column=0, row=3)
        ttk.Label(self.telemetryEKFFrame, text='Head Bias: ').grid(column=0, row=4)
        ttk.Label(self.telemetryEKFFrame, text=' m').grid(column=2, row=0)
        ttk.Label(self.telemetryEKFFrame, text=' m').grid(column=2, row=1)
        ttk.Label(self.telemetryEKFFrame, text=' m/s').grid(column=2, row=2)
        ttk.Label(self.telemetryEKFFrame, text=' m/s').grid(column=2, row=3)
        ttk.Label(self.telemetryEKFFrame, text=' rad').grid(column=2, row=4)
        self.telemetryEkfPosNVarLabel = ttk.Label(self.telemetryEKFFrame, textvariable=self.telemetryEkfPosNVar).grid(column=1, row=0)
        self.telemetryEkfPosEVarLabel = ttk.Label(self.telemetryEKFFrame, textvariable=self.telemetryEkfPosEVar).grid(column=1, row=1)
        self.telemetryEkfVelNVarLabel = ttk.Label(self.telemetryEKFFrame, textvariable=self.telemetryEkfVelNVar).grid(column=1, row=2)
        self.telemetryEkfVelEVarLabel = ttk.Label(self.telemetryEKFFrame, textvariable=self.telemetryEkfVelEVar).grid(column=1, row=3)
        self.telemetryEkfBiasVarLabel = ttk.Label(self.telemetryEKFFrame, textvariable=self.telemetryEkfBiasVar).grid(column=1, row=4)

        # Raw GPS Frame (EXISTING)
        self.telemetryRawGPSFrame = ttk.LabelFrame(self.telemetryFrame, text= "Raw GPS")
        self.telemetryRawGPSFrame.grid(column=1, row=0, ipady=6)
        ttk.Label(self.telemetryRawGPSFrame, text='Raw Lat: ').grid(column=0, row=0)
        ttk.Label(self.telemetryRawGPSFrame, text='Raw Lon: ').grid(column=0, row=1)
        ttk.Label(self.telemetryRawGPSFrame, text='Raw Speed: ').grid(column=0, row=2)
        ttk.Label(self.telemetryRawGPSFrame, text='Raw COG: ').grid(column=0, row=3)
        ttk.Label(self.telemetryRawGPSFrame, text=' °').grid(column=2, row=0)
        ttk.Label(self.telemetryRawGPSFrame, text=' °').grid(column=2, row=1)
        ttk.Label(self.telemetryRawGPSFrame, text=' m/s').grid(column=2, row=2)
        ttk.Label(self.telemetryRawGPSFrame, text=' °').grid(column=2, row=3)
        self.telemetryRawGPSLatVarLabel = ttk.Label(self.telemetryRawGPSFrame, textvariable=self.telemetryRawGPSLatVar).grid(column=1, row=0)
        self.telemetryRawGPSLonVarLabel = ttk.Label(self.telemetryRawGPSFrame, textvariable=self.telemetryRawGPSLonVar).grid(column=1, row=1)
        self.telemetryRawGPSSpeedVarLabel = ttk.Label(self.telemetryRawGPSFrame, textvariable=self.telemetryRawGPSSpeedVar).grid(column=1, row=2)
        self.telemetryRawGPSHeadVarLabel = ttk.Label(self.telemetryRawGPSFrame, textvariable=self.telemetryRawGPSHeadVar).grid(column=1, row=3)

        # Fused State Frame (EXISTING)
        self.telemetryFusedFrame = ttk.LabelFrame(self.telemetryFrame, text = "Fused State (EKF)")
        self.telemetryFusedFrame.grid(column=0, row=1, ipady=6)
        ttk.Label(self.telemetryFusedFrame, text='Fused Lat: ').grid(column=0, row=0)
        ttk.Label(self.telemetryFusedFrame, text='Fused Lon: ').grid(column=0, row=1)
        ttk.Label(self.telemetryFusedFrame, text='Fused Head: ').grid(column=0, row=2)
        ttk.Label(self.telemetryFusedFrame, text=' °').grid(column=2, row=0)
        ttk.Label(self.telemetryFusedFrame, text=' °').grid(column=2, row=1)
        ttk.Label(self.telemetryFusedFrame, text=' °').grid(column=2, row=2)
        self.telemetryFusedLatVarLabel = ttk.Label(self.telemetryFusedFrame, textvariable=self.telemetryFusedLatVar).grid(column=1, row=0)
        self.telemetryFusedLonVarLabel = ttk.Label(self.telemetryFusedFrame, textvariable=self.telemetryFusedLonVar).grid(column=1, row=1)
        self.telemetryFusedHeadVarLabel = ttk.Label(self.telemetryFusedFrame, textvariable=self.telemetryFusedHeadVar).grid(column=1, row=2)

        # ============ ADD THESE NEW FRAMES ============

        # Attitude Frame (Euler Angles) - NEW
        self.telemetryAttitudeFrame = ttk.LabelFrame(self.telemetryFrame, text="Attitude (Euler)")
        self.telemetryAttitudeFrame.grid(column=1, row=1, ipady=6)
        ttk.Label(self.telemetryAttitudeFrame, text='Roll: ').grid(column=0, row=0)
        ttk.Label(self.telemetryAttitudeFrame, text='Pitch: ').grid(column=0, row=1)
        ttk.Label(self.telemetryAttitudeFrame, text='Yaw: ').grid(column=0, row=2)
        ttk.Label(self.telemetryAttitudeFrame, text=' °').grid(column=2, row=0)
        ttk.Label(self.telemetryAttitudeFrame, text=' °').grid(column=2, row=1)
        ttk.Label(self.telemetryAttitudeFrame, text=' °').grid(column=2, row=2)
        self.rollVarLabel = ttk.Label(self.telemetryAttitudeFrame, textvariable=self.rollVar).grid(column=1, row=0)
        self.pitchVarLabel = ttk.Label(self.telemetryAttitudeFrame, textvariable=self.pitchVar).grid(column=1, row=1)
        self.yawVarLabel = ttk.Label(self.telemetryAttitudeFrame, textvariable=self.yawVar).grid(column=1, row=2)

        # Gyroscope Frame - NEW
        self.telemetryGyroFrame = ttk.LabelFrame(self.telemetryFrame, text="Gyroscope (Body)")
        self.telemetryGyroFrame.grid(column=0, row=2, ipady=6)
        ttk.Label(self.telemetryGyroFrame, text='Gyro X: ').grid(column=0, row=0)
        ttk.Label(self.telemetryGyroFrame, text='Gyro Y: ').grid(column=0, row=1)
        ttk.Label(self.telemetryGyroFrame, text='Gyro Z: ').grid(column=0, row=2)
        ttk.Label(self.telemetryGyroFrame, text=' °/s').grid(column=2, row=0)
        ttk.Label(self.telemetryGyroFrame, text=' °/s').grid(column=2, row=1)
        ttk.Label(self.telemetryGyroFrame, text=' °/s').grid(column=2, row=2)
        self.gyroXVarLabel = ttk.Label(self.telemetryGyroFrame, textvariable=self.gyroXVar).grid(column=1, row=0)
        self.gyroYVarLabel = ttk.Label(self.telemetryGyroFrame, textvariable=self.gyroYVar).grid(column=1, row=1)
        self.gyroZVarLabel = ttk.Label(self.telemetryGyroFrame, textvariable=self.gyroZVar).grid(column=1, row=2)

        # Accelerometer Frame - NEW
        self.telemetryAccelFrame = ttk.LabelFrame(self.telemetryFrame, text="Accelerometer (Body)")
        self.telemetryAccelFrame.grid(column=1, row=2, ipady=6)
        ttk.Label(self.telemetryAccelFrame, text='Accel X: ').grid(column=0, row=0)
        ttk.Label(self.telemetryAccelFrame, text='Accel Y: ').grid(column=0, row=1)
        ttk.Label(self.telemetryAccelFrame, text='Accel Z: ').grid(column=0, row=2)
        ttk.Label(self.telemetryAccelFrame, text=' m/s²').grid(column=2, row=0)
        ttk.Label(self.telemetryAccelFrame, text=' m/s²').grid(column=2, row=1)
        ttk.Label(self.telemetryAccelFrame, text=' m/s²').grid(column=2, row=2)
        self.accelXVarLabel = ttk.Label(self.telemetryAccelFrame, textvariable=self.accelXVar).grid(column=1, row=0)
        self.accelYVarLabel = ttk.Label(self.telemetryAccelFrame, textvariable=self.accelYVar).grid(column=1, row=1)
        self.accelZVarLabel = ttk.Label(self.telemetryAccelFrame, textvariable=self.accelZVar).grid(column=1, row=2)

        # Quaternion Frame - NEW
        self.telemetryQuatFrame = ttk.LabelFrame(self.telemetryFrame, text="Quaternion")
        self.telemetryQuatFrame.grid(column=0, row=3, ipady=6, columnspan=2)
        ttk.Label(self.telemetryQuatFrame, text='Quat W: ').grid(column=0, row=0)
        ttk.Label(self.telemetryQuatFrame, text='Quat X: ').grid(column=2, row=0)
        ttk.Label(self.telemetryQuatFrame, text='Quat Y: ').grid(column=4, row=0)
        self.quatWVarLabel = ttk.Label(self.telemetryQuatFrame, textvariable=self.quatWVar).grid(column=1, row=0, padx=5)
        self.quatXVarLabel = ttk.Label(self.telemetryQuatFrame, textvariable=self.quatXVar).grid(column=3, row=0, padx=5)
        self.quatYVarLabel = ttk.Label(self.telemetryQuatFrame, textvariable=self.quatYVar).grid(column=5, row=0, padx=5)
        
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
        expected_len = 97
        if len(rxData) != expected_len:
            self._log(f"RX: Received malformed packet, len={len(rxData)}. Flushing buffer.")
            self._log(f"Data: {rxData.hex()}")
            self.serial_port.reset_input_buffer() # Clear buffer on error
            return
            
        payload = struct.unpack('<24f', rxData[0:expected_len-1])
        
        # New EKF Debug Payload Mapping
        # [0-2] Fused State
        self.telemetryFusedLatVar.set(round(payload[0], 6))
        self.telemetryFusedLonVar.set(round(payload[1], 6))
        self.telemetryFusedHeadVar.set(round(payload[2], 2))

        # [3-7] EKF Internal State
        self.telemetryEkfPosNVar.set(round(payload[3], 3))
        self.telemetryEkfPosEVar.set(round(payload[4], 3))
        self.telemetryEkfVelNVar.set(round(payload[5], 3))
        self.telemetryEkfVelEVar.set(round(payload[6], 3))
        self.telemetryEkfBiasVar.set(round(payload[7], 3))
        
        # [8-11] Raw GPS Data
        self.telemetryRawGPSLatVar.set(round(payload[8], 6))
        self.telemetryRawGPSLonVar.set(round(payload[9], 6))
        self.telemetryRawGPSSpeedVar.set(round(payload[10], 3))
        self.telemetryRawGPSHeadVar.set(round(payload[11], 2))
        
        # [12-14] Euler Angles (NEW)
        self.rollVar.set(round(payload[12], 2))
        self.pitchVar.set(round(payload[13], 2))
        self.yawVar.set(round(payload[14], 2))
        
        # [15-17] Gyroscope (NEW)
        self.gyroXVar.set(round(payload[15], 2))
        self.gyroYVar.set(round(payload[16], 2))
        self.gyroZVar.set(round(payload[17], 2))
        
        # [18-20] Accelerometer (NEW)
        self.accelXVar.set(round(payload[18], 3))
        self.accelYVar.set(round(payload[19], 3))
        self.accelZVar.set(round(payload[20], 3))
        
        # [21-23] Quaternion (NEW)
        self.quatWVar.set(round(payload[21], 4))
        self.quatXVar.set(round(payload[22], 4))
        self.quatYVar.set(round(payload[23], 4))
        
        self._log(f"RX: Euler(R={payload[12]:.1f}°, P={payload[13]:.1f}°, Y={payload[14]:.1f}°) | "
                  f"Gyro({payload[15]:.1f}, {payload[16]:.1f}, {payload[17]:.1f}) | "
                  f"Fused(Lat={payload[0]:.5f}, Lon={payload[1]:.5f}, H={payload[2]:.1f}°)")
        
    def _periodic_ui_update(self):

        # Run GUI Checks
        self.toggleModeCheck()
        
        # If we are currently connected, check for and process incoming data
        if self.serial_port and self.serial_port.is_open:
            try:
                # Process all complete packets in the buffer
                while self.serial_port.in_waiting >= 97: # 49 bytes = 1 full packet
                    self._rxSensorData()
            except Exception as e:
                self._log(f"Error in periodic update: {e}")
                self.disconnect() # Disconnect if the port fails

        # Schedule the next update
        self.after(GUI_UPDATE_PERIOD, self._periodic_ui_update)
        self._update_map()

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


    def _update_map(self):
        """Draw 2D map of EKF position + heading."""

        # Get N/E position
        n = self.telemetryEkfPosNVar.get()
        e = self.telemetryEkfPosEVar.get()
        heading_deg = self.telemetryFusedHeadVar.get()

        # Append to trail
        self.path_north.append(n)
        self.path_east.append(e)

        # Keep trail short
        if len(self.path_north) > 500:
            self.path_north.pop(0)
            self.path_east.pop(0)

        # Update the path line
        self.path_line.set_xdata(self.path_east)
        self.path_line.set_ydata(self.path_north)

        # Remove old heading arrow
        if self.heading_arrow:
            self.heading_arrow.remove()

        # Convert heading to radians (NED: 0° = North)
        theta = np.radians(heading_deg)

        # Heading arrow scale
        arrow_len = 5.0
        dx = arrow_len * np.sin(theta)
        dy = arrow_len * np.cos(theta)

        # Draw new arrow
        self.heading_arrow = self.ax.arrow(
            e, n,        # tail
            dx, dy,      # vector
            width=0.5,
            head_width=2.0,
            head_length=3.0,
            color="red"
        )

        # Auto-scale plot
        self.ax.relim()
        self.ax.autoscale_view()

        # Redraw
        self.canvas.draw()

    

    def on_close(self):
        self.disconnect()
        self.destroy()

if __name__ == "__main__":
    app = XBeeDashboard()
    app.mainloop()