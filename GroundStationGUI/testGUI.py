#!/usr/bin/env python3
"""
xbee_dashboard.py
Requires: pyserial, matplotlib
pip install pyserial matplotlib
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

DEFAULT_BAUD = 9600
PLOT_POINTS = 100

class XBeeDashboard(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("XBee Dashboard")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.serial_port = None
        self.stop_event = threading.Event()
        self.rx_thread = None

        # Telemetry values
        self.telemetry = {"TEMP": "N/A", "HUM": "N/A", "BAT": "N/A", "ALTITUDE": "N/A"}
        self.temp_history = deque(maxlen=PLOT_POINTS)

        # Control Mode Radiobutton Labels
        self.modes = ["Manual", "Auton"]
        # Create a shared control variable for both Radiobuttons
        self.modeVar = tk.StringVar(value="manual")

        self._build_ui()
        self.after(500, self._periodic_ui_update)

    def _build_ui(self):
        # Main Frame: This will be the top-most level Frame of the GUI
        # (This can be thought of as the "root node" of our tree of frames)

        # main = ttk.Frame(self)

        # Top frame: connection controls
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
        left = ttk.Frame(self)
        left.grid(column=0, row=1)
        # left.pack(side="")
        # Middle frame: dashboard + console + button commands
        middle = ttk.Frame(self)
        # middle.pack(side="top", fill="both", expand=True, padx=8, pady=6)
        middle.grid(column=1, row=1)

        # Reserve the following spots in the grid for each frame:
        # For Dashboard and DashboardTwo, use column = 0, and then rows=0 and 1
        # For Console use column = 1 and rows = 0
        # Todo: For Console buttons and commands, make a seperate frame and put it at column = 1, rows = 1
        # Todo: For Map Frame, place it at column = 2 and rows = 0


        # Left-Middle frame: dashboard widgets
        dash_frame = ttk.LabelFrame(middle, text="Telemetry")
        # dash_frame.pack(side="left", fill="y", padx=(0,8))
        dash_frame.grid(column=0, row=0, ipady=12)

        row = 0
        for key in ["TEMP", "HUM", "BAT", "ALTITUDE"]:
            ttk.Label(dash_frame, text=f"{key}:", font=("TkDefaultFont", 10)).grid(row=row, column=0, sticky="w", padx=6, pady=6)
            lbl = ttk.Label(dash_frame, text=self.telemetry[key], font=("TkDefaultFont", 12, "bold"))
            lbl.grid(row=row, column=1, sticky="w", padx=6, pady=6)
            setattr(self, f"lbl_{key}", lbl)
            row += 1

        # Left Two: mode widgets
        dash_frame_two = ttk.LabelFrame(middle, text="Modes")
        # dash_frame_two.pack(side="left", padx=(0,8))
        dash_frame_two.grid(column=1, row=0)



        manual = ttk.Radiobutton(dash_frame_two, text=self.modes[0], variable=self.modeVar, value="manual")
        manual.grid(row=1, column=2, sticky="w", padx=6, pady=6)
        # lbl_manual = ttk.Label(dash_frame_two, text=self.modes[0], font=("TkDefaultFont", 12, "normal"))
        # lbl_manual.grid(row=4, column=1, sticky="w", padx=6, pady=6)
        auton = ttk.Radiobutton(dash_frame_two, text=self.modes[1], variable=self.modeVar, value="auton")
        auton.grid(row=2, column=2, sticky="w", padx=6, pady=6)
        # lbl_auton = ttk.Label(dash_frame_two, text=self.modes[1], font=("TkDefaultFont", 12, "normal"))
        # lbl_auton.grid(row=5, column=1, sticky="w", padx=6, pady=6)

        # Right: Map and other Data
        right = ttk.Frame(self)
        # right.pack(side="left", fill="both", expand=True)
        right.grid(row=1, column=3)
        console_frame = ttk.LabelFrame(right, text="Console")
        console_frame.pack(side="top", fill="both", expand=True)

        self.console = scrolledtext.ScrolledText(console_frame, height=12, state="disabled", wrap="none")
        self.console.pack(fill="both", expand=True, padx=4, pady=4)


        # Command button to send the laptop's GPS coordinates
        # button = ttk.Button(parent, text='Okay', command=submitForm)

        # Todo: Uncomment and fix
        # sendGPSButton = ttk.Button(middle, text='Send Current GPS Coordinates');
        #sendGPSButton.pack()
        # sendGPSButton.grid(column=)
        
        # Temperature Plot Feature from GPT (Could change to something else?)
        # Plot area
        # plot_frame = ttk.LabelFrame(right, text="Temperature (last values)")
        # plot_frame.pack(side="top", fill="both", expand=True, pady=(6,0))

        # self.fig = Figure(figsize=(5,2.2))
        # self.ax = self.fig.add_subplot(111)
        # self.ax.set_title("Temperature")
        # self.ax.set_xlabel("samples")
        # self.ax.set_ylabel("°C")
        # self.line, = self.ax.plot([], [])
        # self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        # self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # Bottom: manual send
        # Parafoil: This will be where we send packets / specific characters to the payload
        # bottom = ttk.Frame(self)
        # bottom.pack(side="bottom", fill="x", padx=8, pady=6)

        #ttk.Label(bottom, text="Send Packet:").pack(side="left")

        # Todo: uncomment and rewrite
        # self.send_btn = ttk.Button(middle, text="Send Packet", command=self._send_text)
        # self.send_btn.pack(side="left")

        # self.send_entry = ttk.Entry(middle, width=13)
        # self.send_entry.pack(side="left", padx=6)
        # self.send_entry.bind("<Return>", lambda e: self._send_text())

        

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
            self.serial_port = serial.Serial(port, baud, timeout=0.5)
        except Exception as e:
            messagebox.showerror("Open port failed", f"Could not open {port}: {e}")
            return

        self.connect_btn.config(state="disabled")
        self.disconnect_btn.config(state="normal")
        self.scan_btn.config(state="disabled")
        self._log(f"Connected to {port} @ {baud}")
        self.stop_event.clear()
        self.rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
        self.rx_thread.start()

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
        self._log("Disconnected")

    def _rx_worker(self):
        buff = bytearray()
        while not self.stop_event.is_set():
            try:
                if self.serial_port is None:
                    break
                data = self.serial_port.read(128)
                if data:
                    buff.extend(data)
                    # handle lines
                    while b'\n' in buff:
                        idx = buff.index(b'\n')
                        line = buff[:idx+1].decode(errors='replace').strip()
                        buff = buff[idx+1:]
                        self._handle_line(line)
                else:
                    time.sleep(0.01)
            except Exception as e:
                self._log(f"Serial read error: {e}")
                time.sleep(0.5)

    def _handle_line(self, line):
        # Display raw line
        self._log("RX: " + line)

        # Expect telemetry as KEY=VALUE;KEY=VALUE;...\n
        try:
            parts = line.strip().split(';')
            changed = False
            for p in parts:
                if '=' in p:
                    k,v = p.split('=',1)
                    k=k.strip().upper()
                    v=v.strip()
                    if k in self.telemetry:
                        self.telemetry[k] = v
                        changed = True
                        if k == "TEMP":
                            try:
                                self.temp_history.append(float(v))
                            except:
                                pass
            if changed:
                self._update_dashboard_widgets()
        except Exception as e:
            # not telemetry or parse error — ignore for dashboard
            pass

    def _log(self, text):
        # append to console thread-safely using after
        ts = time.strftime("%H:%M:%S")
        self.console.configure(state="normal")
        self.console.insert("end", f"[{ts}] {text}\n")
        self.console.see("end")
        self.console.configure(state="disabled")

    def _update_dashboard_widgets(self):
        self.lbl_TEMP.config(text=str(self.telemetry["TEMP"]))
        self.lbl_HUM.config(text=str(self.telemetry["HUM"]))
        self.lbl_BAT.config(text=str(self.telemetry["BAT"]))

    def _periodic_ui_update(self):
        # update plot
        data = list(self.temp_history)
        if data:
            self.line.set_data(range(len(data)), data)
            self.ax.set_xlim(0, max(len(data)-1, PLOT_POINTS))
            ymin = min(data)
            ymax = max(data)
            if ymin == ymax:
                ymin -= 0.5
                ymax += 0.5
            self.ax.set_ylim(ymin, ymax)
        else:
            self.line.set_data([], [])
        self.canvas.draw_idle()
        self.after(500, self._periodic_ui_update)

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
