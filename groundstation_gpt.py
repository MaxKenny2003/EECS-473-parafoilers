"""
xbee_dashboard.py
(Upgraded with Matplotlib plots for EKF visualization)
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import time
from collections import deque
import struct
import numpy as np

# --- Matplotlib Imports ---
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
# --- End Matplotlib Imports ---


DEFAULT_BAUD = 9600
PLOT_POINTS = 100 # How many points to show on the time-series plot
GUI_UPDATE_PERIOD = 100 # Update GUI 10 times a second (100ms)


class XBeeDashboard(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Groundstation XBee Dashboard (EKF Visualizer)")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.serial_port = None
        self.stop_event = threading.Event()
        self.rx_thread = None

        # --- Data Deques for Plotting ---
        self.plot_points = PLOT_POINTS
        self.time_history = deque(maxlen=self.plot_points)
        self.bias_history = deque(maxlen=self.plot_points)
        self.pos_n_history = deque(maxlen=self.plot_points)
        self.pos_e_history = deque(maxlen=self.plot_points)

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

        # [24-25] Absolute Height
        self.adjustAltitudeVar = tk.DoubleVar()
        self.absoluteAltitudeVar = tk.DoubleVar() # Display the Distance between the Structure and the Ground (as opposed to Sea Level)
        # Initalize constant offset based on testing location
        # North Campus: 280 Meters above Sealevel
        self.adjustAltitudeVar.set(280.0)
        # --- Append data to plot deques ---
        # Control Mode Radiobutton Labels
        self.modes = ["Auton", "Manual"]
        # Create a shared control variable for both Radiobuttons
        self.modeVar = tk.StringVar(value="Auton")
        self.manualFrameDrawn = False;

        # Create Entry Variables for the Send GPS Coords Button
        self.gpsLatitudeVar = tk.DoubleVar()
        self.gpsLongitudeVar = tk.DoubleVar()
        
        # Target position for map
        self.target_lat = 0.0
        self.target_lon = 0.0

        # Build UI and then Disable Widgets until we connect to Serial
        self._build_ui()
        self._init_disable()
        self.after(500, self._periodic_ui_update)

    def _build_ui(self):
        # --- Configure root window scaling ---
        self.rowconfigure(1, weight=1)      # Make row 1 (main content) expandable
        self.columnconfigure(0, weight=1)   # Left column (telemetry)
        self.columnconfigure(1, weight=1)   # Middle column (console)
        self.columnconfigure(2, weight=2)   # Right column (plots) - give more weight

        #***************************************#
        # Top frame: Serial Connection Controls #
        #***************************************#

        top = ttk.LabelFrame(self, text="Connection")
        top.grid(column=0, row=0, columnspan=3, sticky="ew", padx=8, pady=6); # Span all columns

        ttk.Label(top, text="Port:").pack(side="left", padx=(5,2))
        self.port_cb = ttk.Combobox(top, width=15, values=self._scan_ports())
        self.port_cb.pack(side="left", padx=4)
        self.port_cb.set(self.port_cb['values'][0] if self.port_cb['values'] else "")

        ttk.Label(top, text="Baud:").pack(side="left", padx=(10,0))
        self.baud_cb = ttk.Combobox(top, width=8, values=[9600, 19200, 38400, 57600, 115200])
        self.baud_cb.pack(side="left", padx=4)
        self.baud_cb.set(DEFAULT_BAUD)

        self.scan_btn = ttk.Button(top, text="Scan Ports", command=self._do_scan)
        self.scan_btn.pack(side="left", padx=6)

        self.connect_btn = ttk.Button(top, text="Connect", command=self.connect)
        self.connect_btn.pack(side="left", padx=6)

        self.disconnect_btn = ttk.Button(top, text="Disconnect", command=self.disconnect, state="disabled")
        self.disconnect_btn.pack(side="left", padx=2)


        #**********************************************#
        # Left frame: telemetry widgets and data#
        #**********************************************#
        
        self.left = ttk.Frame(self)
        self.left.grid(column=0, row=1, sticky="nsew", padx=(8,0)) # Stick to all sides
        self.left.rowconfigure(0, weight=1) # Make telemetry frame expandable
        
        # Telemetry Frames
        self.telemetryFrame = ttk.LabelFrame(self.left, text="Telemetry")
        self.telemetryFrame.pack(fill="both", expand=True) # Use pack to fill the left frame
        self.telemetryFrame.columnconfigure(0, weight=1) # Make columns expandable
        self.telemetryFrame.columnconfigure(1, weight=1)

        # EKF State Frame
        self.telemetryEKFFrame = ttk.LabelFrame(self.telemetryFrame, text= "EKF State (NED)")
        # self.telemetryEKFFrame.grid(column=0, row=0, ipady=6, sticky="nsew", padx=5, pady=5)
        self.telemetryEKFFrame.grid(column=0, row=0, ipady=6, ipadx=15, sticky="nsew", padx=5, pady=5)
        ttk.Label(self.telemetryEKFFrame, text='Pos North: ').grid(column=0, row=0, sticky="w")
        ttk.Label(self.telemetryEKFFrame, text='Pos East: ').grid(column=0, row=1, sticky="w")
        ttk.Label(self.telemetryEKFFrame, text='Vel North: ').grid(column=0, row=2, sticky="w")
        ttk.Label(self.telemetryEKFFrame, text='Vel East: ').grid(column=0, row=3, sticky="w")
        ttk.Label(self.telemetryEKFFrame, text='Head Bias: ').grid(column=0, row=4, sticky="w")
        ttk.Label(self.telemetryEKFFrame, textvariable=self.telemetryEkfPosNVar).grid(column=1, row=0, sticky="e")
        ttk.Label(self.telemetryEKFFrame, textvariable=self.telemetryEkfPosEVar).grid(column=1, row=1, sticky="e")
        ttk.Label(self.telemetryEKFFrame, textvariable=self.telemetryEkfVelNVar).grid(column=1, row=2, sticky="e")
        ttk.Label(self.telemetryEKFFrame, textvariable=self.telemetryEkfVelEVar).grid(column=1, row=3, sticky="e")
        ttk.Label(self.telemetryEKFFrame, textvariable=self.telemetryEkfBiasVar).grid(column=1, row=4, sticky="e")
        ttk.Label(self.telemetryEKFFrame, text=' m').grid(column=2, row=0, sticky="w")
        ttk.Label(self.telemetryEKFFrame, text=' m').grid(column=2, row=1, sticky="w")
        ttk.Label(self.telemetryEKFFrame, text=' m/s').grid(column=2, row=2, sticky="w")
        ttk.Label(self.telemetryEKFFrame, text=' m/s').grid(column=2, row=3, sticky="w")
        ttk.Label(self.telemetryEKFFrame, text=' rad').grid(column=2, row=4, sticky="w")
        self.telemetryEKFFrame.columnconfigure(1, weight=1)

        # Raw GPS Frame
        self.telemetryRawGPSFrame = ttk.LabelFrame(self.telemetryFrame, text= "Raw GPS")
        self.telemetryRawGPSFrame.grid(column=1, row=0, ipady=6, sticky="nsew", padx=5, pady=5)
        ttk.Label(self.telemetryRawGPSFrame, text='Raw Lat: ').grid(column=0, row=0, sticky="w")
        ttk.Label(self.telemetryRawGPSFrame, text='Raw Lon: ').grid(column=0, row=1, sticky="w")
        ttk.Label(self.telemetryRawGPSFrame, text='Raw Speed: ').grid(column=0, row=2, sticky="w")
        ttk.Label(self.telemetryRawGPSFrame, text='Raw COG: ').grid(column=0, row=3, sticky="w")
        ttk.Label(self.telemetryRawGPSFrame, textvariable=self.telemetryRawGPSLatVar).grid(column=1, row=0, sticky="e")
        ttk.Label(self.telemetryRawGPSFrame, textvariable=self.telemetryRawGPSLonVar).grid(column=1, row=1, sticky="e")
        ttk.Label(self.telemetryRawGPSFrame, textvariable=self.telemetryRawGPSSpeedVar).grid(column=1, row=2, sticky="e")
        ttk.Label(self.telemetryRawGPSFrame, textvariable=self.telemetryRawGPSHeadVar).grid(column=1, row=3, sticky="e")
        ttk.Label(self.telemetryRawGPSFrame, text=' °').grid(column=2, row=0, sticky="w")
        ttk.Label(self.telemetryRawGPSFrame, text=' °').grid(column=2, row=1, sticky="w")
        ttk.Label(self.telemetryRawGPSFrame, text=' m/s').grid(column=2, row=2, sticky="w")
        ttk.Label(self.telemetryRawGPSFrame, text=' °').grid(column=2, row=3, sticky="w")
        self.telemetryRawGPSFrame.columnconfigure(1, weight=1)

        # Fused State Frame
        self.telemetryFusedFrame = ttk.LabelFrame(self.telemetryFrame, text = "Fused State (EKF)")
        self.telemetryFusedFrame.grid(column=0, row=1, ipady=6, sticky="nsew", padx=5, pady=5)
        ttk.Label(self.telemetryFusedFrame, text='Fused Lat: ').grid(column=0, row=0, sticky="w")
        ttk.Label(self.telemetryFusedFrame, text='Fused Lon: ').grid(column=0, row=1, sticky="w")
        ttk.Label(self.telemetryFusedFrame, text='Fused Head: ').grid(column=0, row=2, sticky="w")
        ttk.Label(self.telemetryFusedFrame, textvariable=self.telemetryFusedLatVar).grid(column=1, row=0, sticky="e")
        ttk.Label(self.telemetryFusedFrame, textvariable=self.telemetryFusedLonVar).grid(column=1, row=1, sticky="e")
        ttk.Label(self.telemetryFusedFrame, textvariable=self.telemetryFusedHeadVar).grid(column=1, row=2, sticky="e")
        ttk.Label(self.telemetryFusedFrame, text=' °').grid(column=2, row=0, sticky="w")
        ttk.Label(self.telemetryFusedFrame, text=' °').grid(column=2, row=1, sticky="w")
        ttk.Label(self.telemetryFusedFrame, text=' °').grid(column=2, row=2, sticky="w")
        self.telemetryFusedFrame.columnconfigure(1, weight=1)

        # Attitude Frame (Euler Angles)
        self.telemetryAttitudeFrame = ttk.LabelFrame(self.telemetryFrame, text="Attitude (Euler)")
        self.telemetryAttitudeFrame.grid(column=1, row=1, ipady=6, sticky="nsew", padx=5, pady=5)
        ttk.Label(self.telemetryAttitudeFrame, text='Roll: ').grid(column=0, row=0, sticky="w")
        ttk.Label(self.telemetryAttitudeFrame, text='Pitch: ').grid(column=0, row=1, sticky="w")
        ttk.Label(self.telemetryAttitudeFrame, text='Yaw: ').grid(column=0, row=2, sticky="w")
        ttk.Label(self.telemetryAttitudeFrame, textvariable=self.rollVar).grid(column=1, row=0, sticky="e")
        ttk.Label(self.telemetryAttitudeFrame, textvariable=self.pitchVar).grid(column=1, row=1, sticky="e")
        ttk.Label(self.telemetryAttitudeFrame, textvariable=self.yawVar).grid(column=1, row=2, sticky="e")
        ttk.Label(self.telemetryAttitudeFrame, text=' °').grid(column=2, row=0, sticky="w")
        ttk.Label(self.telemetryAttitudeFrame, text=' °').grid(column=2, row=1, sticky="w")
        ttk.Label(self.telemetryAttitudeFrame, text=' °').grid(column=2, row=2, sticky="w")
        self.telemetryAttitudeFrame.columnconfigure(1, weight=1)

        # Gyroscope Frame
        self.telemetryGyroFrame = ttk.LabelFrame(self.telemetryFrame, text="Gyroscope (Body)")
        self.telemetryGyroFrame.grid(column=0, row=2, ipady=6, sticky="nsew", padx=5, pady=5)
        ttk.Label(self.telemetryGyroFrame, text='Gyro X: ').grid(column=0, row=0, sticky="w")
        ttk.Label(self.telemetryGyroFrame, text='Gyro Y: ').grid(column=0, row=1, sticky="w")
        ttk.Label(self.telemetryGyroFrame, text='Gyro Z: ').grid(column=0, row=2, sticky="w")
        ttk.Label(self.telemetryGyroFrame, textvariable=self.gyroXVar).grid(column=1, row=0, sticky="e")
        ttk.Label(self.telemetryGyroFrame, textvariable=self.gyroYVar).grid(column=1, row=1, sticky="e")
        ttk.Label(self.telemetryGyroFrame, textvariable=self.gyroZVar).grid(column=1, row=2, sticky="e")
        ttk.Label(self.telemetryGyroFrame, text=' °/s').grid(column=2, row=0, sticky="w")
        ttk.Label(self.telemetryGyroFrame, text=' °/s').grid(column=2, row=1, sticky="w")
        ttk.Label(self.telemetryGyroFrame, text=' °/s').grid(column=2, row=2, sticky="w")
        self.telemetryGyroFrame.columnconfigure(1, weight=1)

        # Accelerometer Frame
        self.telemetryAccelFrame = ttk.LabelFrame(self.telemetryFrame, text="Accelerometer (Body)")
        self.telemetryAccelFrame.grid(column=1, row=2, ipady=6, sticky="nsew", padx=5, pady=5)
        ttk.Label(self.telemetryAccelFrame, text='Accel X: ').grid(column=0, row=0, sticky="w")
        ttk.Label(self.telemetryAccelFrame, text='Accel Y: ').grid(column=0, row=1, sticky="w")
        ttk.Label(self.telemetryAccelFrame, text='Accel Z: ').grid(column=0, row=2, sticky="w")
        ttk.Label(self.telemetryAccelFrame, textvariable=self.accelXVar).grid(column=1, row=0, sticky="e")
        ttk.Label(self.telemetryAccelFrame, textvariable=self.accelYVar).grid(column=1, row=1, sticky="e")
        ttk.Label(self.telemetryAccelFrame, textvariable=self.accelZVar).grid(column=1, row=2, sticky="e")
        ttk.Label(self.telemetryAccelFrame, text=' m/s²').grid(column=2, row=0, sticky="w")
        ttk.Label(self.telemetryAccelFrame, text=' m/s²').grid(column=2, row=1, sticky="w")
        ttk.Label(self.telemetryAccelFrame, text=' m/s²').grid(column=2, row=2, sticky="w")
        self.telemetryAccelFrame.columnconfigure(1, weight=1)
        
        # Quaternion Frame
        self.telemetryQuatFrame = ttk.LabelFrame(self.telemetryFrame, text="Quaternion")
        self.telemetryQuatFrame.grid(column=0, row=3, ipady=6, columnspan=2, sticky="nsew", padx=5, pady=5)
        ttk.Label(self.telemetryQuatFrame, text='Quat W: ').grid(column=0, row=0, sticky="w")
        ttk.Label(self.telemetryQuatFrame, textvariable=self.quatWVar).grid(column=1, row=0, sticky="e")
        ttk.Label(self.telemetryQuatFrame, text='Quat X: ').grid(column=2, row=0, sticky="w")
        ttk.Label(self.telemetryQuatFrame, textvariable=self.quatXVar).grid(column=3, row=0, sticky="e")
        ttk.Label(self.telemetryQuatFrame, text='Quat Y: ').grid(column=4, row=0, sticky="w")
        ttk.Label(self.telemetryQuatFrame, textvariable=self.quatYVar).grid(column=5, row=0, sticky="e")
        self.telemetryQuatFrame.columnconfigure(1, weight=1)
        self.telemetryQuatFrame.columnconfigure(3, weight=1)
        self.telemetryQuatFrame.columnconfigure(5, weight=1)

        # Absolute Height Frame
        self.telemetryAbsHeightFrame = ttk.LabelFrame(self.telemetryFrame, text="Absolute Height")
        self.telemetryAbsHeightFrame.grid(column=0, row=4, ipady=6, columnspan=2, sticky="nsew", padx=5, pady=5)
        ttk.Label(self.telemetryAbsHeightFrame, text='Absolute Height: ').grid(column=0, row=0, sticky="w")
        ttk.Label(self.telemetryAbsHeightFrame, textvariable=self.absoluteAltitudeVar).grid(column=1, row=0, sticky="e")
        ttk.Label(self.telemetryAbsHeightFrame, text= ' m').grid(column=2, row=0, sticky="w")

        # "Mode" Frame
        self.modesFrame = ttk.LabelFrame(self.left, text="Modes")
        self.modesFrame.pack(fill="x", expand=False, pady=5) # pack this below telemetry
        self.auton = ttk.Radiobutton(self.modesFrame, text=self.modes[0], variable=self.modeVar, value="Auton", command=self.toggleModeCheck)
        self.auton.pack(side="left", padx=10, pady=5)
        self.manual = ttk.Radiobutton(self.modesFrame, text=self.modes[1], variable=self.modeVar, value="Manual", command=self.toggleModeCheck)
        self.manual.pack(side="left", padx=10, pady=5)
        
        # This frame will appear/disappear
        self.controlFrame = ttk.LabelFrame(self.left, text="Manual Controls")
        self.controlFrame.pack(fill="x", expand=False) # pack below modes
        self.turnLeftButton = ttk.Button(self.controlFrame, text="Turn Left", command=self._manualTurnLeft)
        self.turnLeftButton.pack(side="left", fill="x", expand=True, padx=5, pady=5)
        self.turnRightButton = ttk.Button(self.controlFrame, text="Turn Right", command=self._manualTurnRight)
        self.turnRightButton.pack(side="left", fill="x", expand=True, padx=5, pady=5)
        self.controlFrame.pack_forget() # Hide it initially
        self.manualFrameDrawn = False

        
        #***************************************************#
        #Middle frame: dashboard + console + button commands#
        #***************************************************#

        middle = ttk.Frame(self)
        middle.grid(column=1, row=1, sticky="nsew", padx=8) # Use column 1
        middle.rowconfigure(0, weight=1) # Allow console_frame to expand
        middle.columnconfigure(0, weight=1) # Allow console_frame to expand

        console_frame = ttk.LabelFrame(middle, text="Console")
        console_frame.grid(column=0, row=0, sticky="nsew") # make sticky
        console_frame.rowconfigure(0, weight=1) # Add weights
        console_frame.columnconfigure(0, weight=1)

        self.console = scrolledtext.ScrolledText(console_frame, height=12, state="disabled", wrap="none")
        self.console.grid(row=0, column=0, sticky="nsew", padx=4, pady=4) # Use grid
  
        # Console Commands (E-Stop, Send Coordinates)
        self.consoleCommandsFrame = ttk.LabelFrame(middle, text="Commands")
        self.consoleCommandsFrame.grid(column=0, row=1, sticky="ew") # Make sticky
        self.consoleCommandsFrame.columnconfigure(4, weight=1) # allow entry to expand
        
        self.emergencyStopButton = ttk.Button(self.consoleCommandsFrame, text="Emergency Stop / Deadfall", command=self._emergencyStopCmd)
        self.emergencyStopButton.grid(column=0, row=0, columnspan=5, sticky="ew", padx=5, pady=5)

        # Send GPS Coordinates (Button and Entries)
        self.sendGPSButton = ttk.Button(self.consoleCommandsFrame, text='Send Target', command=self._sendGPSCmd)
        self.sendGPSButton.grid(column=0, row=1, padx=5, pady=5)
        ttk.Label(self.consoleCommandsFrame, text='Latitude: ').grid(column=1, row=1)
        self.sendGPSLatitudeEntry = ttk.Entry(self.consoleCommandsFrame, textvariable=self.gpsLatitudeVar, width=10)
        self.sendGPSLatitudeEntry.grid(column=2, row=1)
        ttk.Label(self.consoleCommandsFrame, text='Longitude: ').grid(column=3, row=1, padx=(5,0))
        self.sendGPSLongitudeEntry = ttk.Entry(self.consoleCommandsFrame, textvariable=self.gpsLongitudeVar, width=10)
        self.sendGPSLongitudeEntry.grid(column=4, row=1, padx=(0,5))        
        
        # Reset Plot Button
        self.resetPlotButton = ttk.Button(self.consoleCommandsFrame, text='Reset Plots', command=self._clearPlotData)
        self.resetPlotButton.grid(column=0, row=3)

        # Adjust Alitude Entry
        self.adjustAltitudeLabel = ttk.Label(self.consoleCommandsFrame, text="Subtract Altitude by this value in Meters")
        self.adjustAltitudeLabel.grid(column=0, row=4)
        self.adjustAltitudeEntry = ttk.Entry(self.consoleCommandsFrame, textvariable=self.adjustAltitudeVar)
        self.adjustAltitudeEntry.grid(column=1, row=4)
        # Deployment Button (Add relevant metrics to the side)
        # Todo: Ask Max if there should be some kind of condition before deploying (like only deploy if we're above 100 feet)
        self.deploymentButton = ttk.Button(self.consoleCommandsFrame, text='Deploy Payload', command=self._deployPayloadCmd)
        self.deploymentButton.grid(column=0, row=5)
        
        #***************************************************#
        # Right frame: Plots
        #***************************************************#
        
        plot_frame = ttk.LabelFrame(self, text="Visualization")
        plot_frame.grid(row=1, column=2, sticky="nsew", padx=(0,8))
        plot_frame.rowconfigure(0, weight=1)
        plot_frame.columnconfigure(0, weight=1)

        # Create Figure and Subplots
        self.fig = Figure(figsize=(8, 8), dpi=100, tight_layout=True)
        self.ax_map = self.fig.add_subplot(2, 2, 1) # 2x2 grid, 1st plot
        self.ax_compass = self.fig.add_subplot(2, 2, 2, projection='polar') # 2x2 grid, 2nd plot
        self.ax_bias = self.fig.add_subplot(2, 1, 2) # 2x1 grid, 2nd row (spans bottom)

        # --- Setup Map Plot (ax_map) ---
        self.ax_map.set_title("NED Position (m)")
        self.ax_map.set_xlabel("East (m)")
        self.ax_map.set_ylabel("North (m)")
        self.ax_map.grid(True)
        self.map_line, = self.ax_map.plot([], [], 'bo-', label="Path") # Fused path
        self.map_gps, = self.ax_map.plot([], [], 'kx', label="Raw GPS", markersize=5, alpha=0.5)
        self.map_target, = self.ax_map.plot([], [], 'rX', label="Target", markersize=10)
        self.ax_map.legend(fontsize='small')

        # --- Setup Compass Plot (ax_compass) ---
        self.ax_compass.set_title("Heading (°)", pad=20)
        self.ax_compass.set_theta_zero_location('N') # 0° is North
        self.ax_compass.set_theta_direction(-1) # Clockwise
        self.compass_fused = self.ax_compass.annotate(
            "", xy=(0, 0), xytext=(0, 0),
            arrowprops=dict(facecolor='blue', shrink=0.05, width=2, headwidth=8)
        )
        self.compass_raw = self.ax_compass.annotate(
            "", xy=(0, 0), xytext=(0, 0),
            arrowprops=dict(facecolor='gray', shrink=0.05, width=1, headwidth=6, alpha=0.7)
        )
        self.ax_compass.set_yticklabels([]) # Hide radius labels

        # --- Setup Bias Plot (ax_bias) ---
        self.ax_bias.set_title("EKF Heading Bias (rad)")
        self.ax_bias.set_xlabel("Time (s)")
        self.ax_bias.set_ylabel("Bias (rad)")
        self.ax_bias.grid(True)
        self.bias_line, = self.ax_bias.plot([], [], 'r-', label="Heading Bias")
        
        # Add canvas to tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(side="top", fill="both", expand=True)
        self.canvas.draw()


    def _init_disable(self):
        # Disable several buttons and widgets upon start up
        self.manual.config(state="disabled")
        self.auton.config(state="disabled")
        self.emergencyStopButton.config(state="disabled")
        self.sendGPSButton.config(state="disabled")
        self.sendGPSLatitudeEntry.config(state="disabled")
        self.sendGPSLongitudeEntry.config(state="disabled")
        self.turnLeftButton.config(state="disabled")
        self.turnRightButton.config(state="disabled")
        self.deploymentButton.config(state="disabled")
        self.adjustAltitudeEntry.config(state="disabled")

    def _init_enable(self):
        # Enable several buttons and widegets upon start up
        self.manual.config(state="normal")
        self.auton.config(state="normal")
        self.emergencyStopButton.config(state="normal")
        self.sendGPSButton.config(state="normal")
        self.sendGPSLatitudeEntry.config(state="normal")
        self.sendGPSLongitudeEntry.config(state="normal")
        self.turnLeftButton.config(state="normal")
        self.turnRightButton.config(state="normal")
        self.deploymentButton.config(state="normal")
        self.adjustAltitudeEntry.config(state="normal")
        
        self.modeVar.set("Auton")
        self.toggleModeCheck() # Call this to set the correct initial state
        self._log("Enabling Auton Mode by Default")
        # self._setToAuto() # This is called by toggleModeCheck

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
            self.connect_btn.config(state="disabled")
            self.disconnect_btn.config(state="normal")
            self.scan_btn.config(state="disabled")
            self._init_enable()
            self.serialConnectLog()
            
            # Clear plot history
            self.time_history.clear()
            self.bias_history.clear()
            self.pos_n_history.clear()
            self.pos_e_history.clear()
            
        else:
            self._log("Error: port not opened")

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
    #     ...

    # def _handle_line(self, line):
    #     ...

    #################################################
    # Left-Middle Frame Functions: Telemetry, Modes #
    #################################################

    def toggleModeCheck(self):
        if self.modeVar.get() == "Manual":
            if not self.manualFrameDrawn:
                self.controlFrame.pack(fill="x", expand=False, pady=5) # Show the frame
                self.manualFrameDrawn = True
                self._setToManual()
                self._log("Toggled to Manual Mode")
        else: # Auton mode
            if self.manualFrameDrawn:
                self.controlFrame.pack_forget() # Hide the frame
                self.manualFrameDrawn = False
                self._setToAuto()
                self._log("Toggled to Autonomous Mode")

    def serialConnectLog(self):
        self._log(f"{self.serial_port.name} serial port is opened")
        self._log(f"Connected to {self.serial_port.port} @ {self.serial_port.baudrate}")
        self._log("\nPARAFOIL GROUND STATION")
        self._log("\nCommands:")
        self._log("  coord  - Send target GPS coordinates using the button in 'Commands' Frame (auto mode)")
        self._log("  Latitude Range: [-90, 90], Longitude Range: [-180, 180]")
        self._log("  L      - Manual control: Turn LEFT")
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
        try:
            ts = time.strftime("%H:%M:%S")
            self.console.configure(state="normal")
            self.console.insert("end", f"[{ts}] {text}\n")
            self.console.see("end")
            self.console.configure(state="disabled")
        except Exception as e:
            print(f"Error logging: {e}")

    def _emergencyStopCmd(self):
        data = b'S' + bytes(8)
        self.serial_port.write(data)
        self._log("→ Manual STOP command sent")
        self._log("E-Stoped the Parafoil.")
   
    def consoleValidateCoordinates(self, lat, lon):
        """Validate latitude and longitude ranges."""
        try:
            lat_f = float(lat)
            lon_f = float(lon)
            
            if not (-90 <= lat_f <= 90):
                self._log("Error: Latitude must be between -90 and 90")
                return None, None
            
            if not (-180 <= lon_f <= 180):
                self._log("Error: Longitude must be between -180 and 180")
                return None, None
            
            return lat_f, lon_f
        except ValueError:
            self._log("Error: Invalid number format")
            return None, None
            
    def consoleCoordsToFixedPoint(self, lat, lon, scale=10000000):
        latFixed = int(lat*scale)
        longFixed = int(lon*scale)
        return latFixed, longFixed

    def _sendGPSCmd(self):
        latStr = self.gpsLatitudeVar.get()
        longStr = self.gpsLongitudeVar.get()
        
        latF, longF = self.consoleValidateCoordinates(latStr, longStr)
        if latF is None or longF is None:
            return
            
        # Store for plotting
        self.target_lat = latF
        self.target_lon = longF
        # We can't plot this on the NED map yet, as we don't have
        # the EKF's ref_lat/ref_lon. This is a future improvement.

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
        self._log("Rerouted coordinates to " + str(latF) + "," + str(longF))

    def _clearPlotData(self):
        # Clear plot history via button press
        self.time_history.clear()
        self.bias_history.clear()
        self.pos_n_history.clear()
        self.pos_e_history.clear()
        # Then update the plots
        self._update_plots()

    def _rxSensorData(self):
        expected_len = 97
        if self.serial_port.in_waiting < expected_len:
            return # Not enough data for a full packet

        # Read one full packet
        rxData = self.serial_port.read(expected_len)
        
        if len(rxData) != expected_len or rxData[-1] != 0x0A: # Check length and newline
            self._log(f"RX: Malformed packet, len={len(rxData)}. Flushing.")
            self._log(f"Data: {rxData.hex()}")
            self.serial_port.reset_input_buffer() # Clear buffer on error
            return

        payload = struct.unpack('<24f', rxData[0:expected_len-1])

        # [0-2] Fused State
        self.telemetryFusedLatVar.set(round(payload[0], 6))
        self.telemetryFusedLonVar.set(round(payload[1], 6))
        self.telemetryFusedHeadVar.set(round(payload[2], 2))

        # [3-7] EKF Internal State
        pos_n = round(payload[3], 3)
        pos_e = round(payload[4], 3)
        bias = round(payload[7], 4)
        self.telemetryEkfPosNVar.set(pos_n)
        self.telemetryEkfPosEVar.set(pos_e)
        self.telemetryEkfVelNVar.set(round(payload[5], 3))
        self.telemetryEkfVelEVar.set(round(payload[6], 3))
        self.telemetryEkfBiasVar.set(bias)

        # [8-11] Raw GPS Data
        self.telemetryRawGPSLatVar.set(round(payload[8], 6))
        self.telemetryRawGPSLonVar.set(round(payload[9], 6))
        self.telemetryRawGPSSpeedVar.set(round(payload[10], 3))
        self.telemetryRawGPSHeadVar.set(round(payload[11], 2))

        # [12-14] Euler Angles
        self.rollVar.set(round(payload[12], 2))
        self.pitchVar.set(round(payload[13], 2))
        self.yawVar.set(round(payload[14], 2))
        
        # [15-17] Gyroscope
        self.gyroXVar.set(round(payload[15], 2))
        self.gyroYVar.set(round(payload[16], 2))
        self.gyroZVar.set(round(payload[17], 2))
        
        # [18-20] Accelerometer
        self.accelXVar.set(round(payload[18], 3))
        self.accelYVar.set(round(payload[19], 3))
        self.accelZVar.set(round(payload[20], 3))
        
        # [21-23] Quaternion
        self.quatWVar.set(round(payload[21], 4))
        self.quatXVar.set(round(payload[22], 4))
        self.quatYVar.set(round(payload[23], 4))
        
        if not self.time_history:
             self.start_time = time.time()
             self.time_history.append(0)
        else:
             self.time_history.append(time.time() - self.start_time)
        
        self.bias_history.append(bias)
        self.pos_n_history.append(pos_n)
        self.pos_e_history.append(pos_e)
        # --- End append ---

        self._log(f"RX: Fused H={payload[2]:.1f}° | Bias={payload[7]:.4f} | Pos N={payload[3]:.1f} E={payload[4]:.1f}")
        
    def _update_plots(self):
        if not self.time_history:
            return # No data yet

        # --- 1. Update Bias Plot ---
        self.ax_bias.clear()
        self.ax_bias.grid(True)
        self.ax_bias.set_title("EKF Heading Bias (rad)")
        self.ax_bias.set_xlabel("Time (s)")
        self.ax_bias.set_ylabel("Bias (rad)")
        self.ax_bias.plot(self.time_history, self.bias_history, 'r-')
        if self.bias_history:
            min_bias = min(self.bias_history) - 0.01
            max_bias = max(self.bias_history) + 0.01
            if min_bias != max_bias:
                self.ax_bias.set_ylim(min_bias, max_bias)


        # --- 2. Update Map Plot ---
        self.ax_map.clear()
        self.ax_map.grid(True)
        self.ax_map.set_title("NED Position (m)")
        self.ax_map.set_xlabel("East (m)")
        self.ax_map.set_ylabel("North (m)")
        
        if self.pos_n_history:
            # Plot the path
            self.ax_map.plot(self.pos_e_history, self.pos_n_history, 'b.-', label="Fused Path")
            # Plot the current position
            self.ax_map.plot(self.pos_e_history[-1], self.pos_n_history[-1], 'bo', markersize=8, label="Current")
            
            # Auto-scale the plot
            self.ax_map.axis('equal')
        
        # (Future: Plot target and raw GPS once ref_lat/lon is sent)
        self.ax_map.legend(fontsize='small')


        # --- 3. Update Compass Plot ---
        self.ax_compass.clear()
        self.ax_compass.set_title("Heading (°)", pad=20)
        self.ax_compass.set_theta_zero_location('N')
        self.ax_compass.set_theta_direction(-1)
        self.ax_compass.set_yticklabels([])
        
        fused_heading_rad = np.deg2rad(self.telemetryFusedHeadVar.get())
        raw_heading_rad = np.deg2rad(self.telemetryRawGPSHeadVar.get())

        # Plot Fused Heading (Blue Arrow)
        self.ax_compass.arrow(0, 0, fused_heading_rad, 0.8,
                              width=0.1, head_width=0.2, head_length=0.1,
                              facecolor='blue', edgecolor='blue', label='Fused')
        
        # Plot Raw GPS Heading (Gray Arrow)
        self.ax_compass.arrow(0, 0, raw_heading_rad, 0.6,
                              width=0.05, head_width=0.15, head_length=0.1,
                              facecolor='gray', edgecolor='gray', alpha=0.7, label='Raw GPS')
        self.ax_compass.set_ylim(0, 1) # Set radius limit
        
        # --- Redraw Canvas ---
        self.canvas.draw_idle()

    def _deployPayloadCmd(self):
        self._log("Deploying Payload...")
        # self._log(f"Drop Height") #Todo: change this to relflect proper height later
        # Todo: Double check if this is the right char
        data = b'P' + bytes(8)
        self.serial_port.write(data)

    def _periodic_ui_update(self):
        # Run GUI Checks
        self.toggleModeCheck()

        # If we are currently connected, check for and process incoming data
        if self.serial_port and self.serial_port.is_open:
            try:
                # Process all complete packets in the buffer
                while self.serial_port.in_waiting >= 97:
                    self._rxSensorData()
                
                # Update plots with any new data
                self._update_plots()
                
            except Exception as e:
                self._log(f"Error in periodic update: {e}")
                self.disconnect() # Disconnect if the port fails

        # Schedule the next update
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