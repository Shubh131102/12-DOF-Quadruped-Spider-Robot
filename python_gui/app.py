"""
Spider Quadruped Robot Control GUI

Tkinter-based control interface for 12-DOF quadruped robot.
Features:
- Individual servo control
- Preset poses
- Gait selection
- Real-time status monitoring
- Serial communication with Arduino
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import time
import threading


class ServoController:
    """Serial communication with Arduino"""
    
    def __init__(self):
        self.serial_port = None
        self.connected = False
        
    def connect(self, port, baud_rate=115200):
        """Connect to Arduino via serial"""
        try:
            self.serial_port = serial.Serial(port, baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            self.connected = True
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        if self.serial_port:
            self.serial_port.close()
            self.connected = False
    
    def send_command(self, command):
        """Send command to Arduino"""
        if self.connected and self.serial_port:
            try:
                self.serial_port.write(f"{command}\n".encode())
                return True
            except Exception as e:
                print(f"Send error: {e}")
                return False
        return False
    
    def send_pose(self, angles):
        """Send pose command (12 servo angles)"""
        cmd = "POSE:" + ",".join([str(int(a)) for a in angles])
        return self.send_command(cmd)
    
    def send_gait(self, gait_name):
        """Send gait selection command"""
        return self.send_command(f"GAIT:{gait_name}")
    
    def send_move(self, forward, turn):
        """Send movement command"""
        return self.send_command(f"MOVE:{int(forward)},{int(turn)}")
    
    def emergency_stop(self):
        """Emergency stop"""
        return self.send_command("STOP")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        
        self.title("Spider Quadruped Control")
        self.geometry("800x600")
        self.resizable(True, True)
        
        # Servo controller
        self.controller = ServoController()
        
        # Servo angles (12 servos, neutral at 90°)
        self.servo_angles = [90] * 12
        
        # Servo names
        self.servo_names = [
            "LF Hip", "LF Thigh", "LF Knee",
            "RF Hip", "RF Thigh", "RF Knee",
            "LH Hip", "LH Thigh", "LH Knee",
            "RH Hip", "RH Thigh", "RH Knee"
        ]
        
        # Create UI
        self.create_widgets()
        
    def create_widgets(self):
        """Create all GUI elements"""
        
        # Main container
        main_frame = ttk.Frame(self)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Title
        title = tk.Label(
            main_frame, 
            text="Spider Quadruped Control Panel",
            font=("Segoe UI", 16, "bold")
        )
        title.grid(row=0, column=0, columnspan=3, pady=10)
        
        # Connection frame
        self.create_connection_frame(main_frame)
        
        # Control tabs
        notebook = ttk.Notebook(main_frame)
        notebook.grid(row=2, column=0, columnspan=3, sticky='nsew', pady=10)
        
        # Servo control tab
        servo_frame = ttk.Frame(notebook)
        notebook.add(servo_frame, text="Servo Control")
        self.create_servo_control(servo_frame)
        
        # Pose control tab
        pose_frame = ttk.Frame(notebook)
        notebook.add(pose_frame, text="Poses")
        self.create_pose_control(pose_frame)
        
        # Gait control tab
        gait_frame = ttk.Frame(notebook)
        notebook.add(gait_frame, text="Gaits")
        self.create_gait_control(gait_frame)
        
        # Movement control tab
        move_frame = ttk.Frame(notebook)
        notebook.add(move_frame, text="Movement")
        self.create_movement_control(move_frame)
        
        # Status bar
        self.status_label = tk.Label(
            main_frame,
            text="Status: Disconnected",
            relief=tk.SUNKEN,
            anchor=tk.W,
            fg="red"
        )
        self.status_label.grid(row=3, column=0, columnspan=3, sticky='ew', pady=5)
        
        # Configure grid weights
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.columnconfigure(2, weight=1)
        main_frame.rowconfigure(2, weight=1)
    
    def create_connection_frame(self, parent):
        """Create connection controls"""
        conn_frame = ttk.LabelFrame(parent, text="Connection", padding=10)
        conn_frame.grid(row=1, column=0, columnspan=3, sticky='ew', pady=5)
        
        # Port selection
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5)
        
        self.port_var = tk.StringVar()
        port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        port_combo['values'] = self.get_serial_ports()
        port_combo.grid(row=0, column=1, padx=5)
        
        # Refresh button
        refresh_btn = ttk.Button(
            conn_frame,
            text="Refresh",
            command=self.refresh_ports
        )
        refresh_btn.grid(row=0, column=2, padx=5)
        
        # Connect button
        self.connect_btn = ttk.Button(
            conn_frame,
            text="Connect",
            command=self.toggle_connection
        )
        self.connect_btn.grid(row=0, column=3, padx=5)
        
        # Emergency stop
        stop_btn = ttk.Button(
            conn_frame,
            text="EMERGENCY STOP",
            command=self.emergency_stop,
            style="Danger.TButton"
        )
        stop_btn.grid(row=0, column=4, padx=5)
    
    def create_servo_control(self, parent):
        """Create individual servo controls"""
        canvas = tk.Canvas(parent)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Create sliders for each servo
        self.servo_sliders = []
        self.servo_labels = []
        
        for i in range(12):
            frame = ttk.Frame(scrollable_frame)
            frame.pack(fill=tk.X, padx=10, pady=5)
            
            # Label
            label = ttk.Label(frame, text=f"{self.servo_names[i]}: 90°", width=15)
            label.pack(side=tk.LEFT, padx=5)
            self.servo_labels.append(label)
            
            # Slider
            slider = ttk.Scale(
                frame,
                from_=0,
                to=180,
                orient=tk.HORIZONTAL,
                command=lambda val, idx=i: self.update_servo(idx, val)
            )
            slider.set(90)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            self.servo_sliders.append(slider)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
    
    def create_pose_control(self, parent):
        """Create preset pose buttons"""
        poses_frame = ttk.Frame(parent, padding=20)
        poses_frame.pack(fill=tk.BOTH, expand=True)
        
        poses = {
            "Neutral": [90] * 12,
            "Standing": [90, 60, 120] * 4,
            "Crouched": [90, 120, 60] * 4,
            "Sit": [90, 45, 135, 90, 135, 45, 90, 135, 45, 90, 45, 135],
            "Wave": [45, 60, 120, 90, 60, 120, 90, 60, 120, 90, 60, 120],
        }
        
        row = 0
        col = 0
        for pose_name, angles in poses.items():
            btn = ttk.Button(
                poses_frame,
                text=pose_name,
                command=lambda a=angles: self.send_pose(a),
                width=15
            )
            btn.grid(row=row, column=col, padx=10, pady=10)
            col += 1
            if col > 2:
                col = 0
                row += 1
    
    def create_gait_control(self, parent):
        """Create gait selection controls"""
        gait_frame = ttk.Frame(parent, padding=20)
        gait_frame.pack(fill=tk.BOTH, expand=True)
        
        ttk.Label(gait_frame, text="Select Gait:", font=("Segoe UI", 12)).pack(pady=10)
        
        gaits = ["walk", "trot", "crawl"]
        
        for gait in gaits:
            btn = ttk.Button(
                gait_frame,
                text=gait.capitalize(),
                command=lambda g=gait: self.send_gait(g),
                width=20
            )
            btn.pack(pady=5)
        
        # Speed control
        ttk.Label(gait_frame, text="Speed:", font=("Segoe UI", 12)).pack(pady=10)
        
        self.speed_var = tk.DoubleVar(value=1.0)
        speed_slider = ttk.Scale(
            gait_frame,
            from_=0.5,
            to=2.0,
            orient=tk.HORIZONTAL,
            variable=self.speed_var,
            command=self.update_speed
        )
        speed_slider.pack(fill=tk.X, padx=50)
        
        self.speed_label = ttk.Label(gait_frame, text="1.0x")
        self.speed_label.pack(pady=5)
    
    def create_movement_control(self, parent):
        """Create movement joystick controls"""
        move_frame = ttk.Frame(parent, padding=20)
        move_frame.pack(fill=tk.BOTH, expand=True)
        
        ttk.Label(
            move_frame,
            text="Movement Control",
            font=("Segoe UI", 12, "bold")
        ).pack(pady=10)
        
        # Forward/Backward
        ttk.Label(move_frame, text="Forward/Backward:").pack()
        self.forward_var = tk.IntVar(value=0)
        forward_slider = ttk.Scale(
            move_frame,
            from_=-100,
            to=100,
            orient=tk.HORIZONTAL,
            variable=self.forward_var
        )
        forward_slider.pack(fill=tk.X, padx=50, pady=5)
        
        # Turn Left/Right
        ttk.Label(move_frame, text="Turn Left/Right:").pack(pady=(20, 0))
        self.turn_var = tk.IntVar(value=0)
        turn_slider = ttk.Scale(
            move_frame,
            from_=-100,
            to=100,
            orient=tk.HORIZONTAL,
            variable=self.turn_var
        )
        turn_slider.pack(fill=tk.X, padx=50, pady=5)
        
        # Send button
        send_btn = ttk.Button(
            move_frame,
            text="Send Movement",
            command=self.send_movement
        )
        send_btn.pack(pady=20)
        
        # Stop button
        stop_btn = ttk.Button(
            move_frame,
            text="Stop Movement",
            command=lambda: self.controller.send_move(0, 0)
        )
        stop_btn.pack()
    
    def get_serial_ports(self):
        """Get list of available serial ports"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def refresh_ports(self):
        """Refresh serial port list"""
        ports = self.get_serial_ports()
        for widget in self.winfo_children():
            if isinstance(widget, ttk.Combobox):
                widget['values'] = ports
        self.update_status("Ports refreshed", "blue")
    
    def toggle_connection(self):
        """Connect/disconnect from Arduino"""
        if not self.controller.connected:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Error", "Please select a port")
                return
            
            if self.controller.connect(port):
                self.connect_btn.config(text="Disconnect")
                self.update_status("Connected", "green")
            else:
                messagebox.showerror("Error", "Failed to connect")
        else:
            self.controller.disconnect()
            self.connect_btn.config(text="Connect")
            self.update_status("Disconnected", "red")
    
    def update_servo(self, index, value):
        """Update individual servo angle"""
        angle = int(float(value))
        self.servo_angles[index] = angle
        self.servo_labels[index].config(text=f"{self.servo_names[index]}: {angle}°")
        
        # Send to robot if connected
        if self.controller.connected:
            self.controller.send_pose(self.servo_angles)
    
    def send_pose(self, angles):
        """Send preset pose"""
        if not self.controller.connected:
            messagebox.showwarning("Warning", "Not connected to robot")
            return
        
        self.servo_angles = angles.copy()
        
        # Update sliders
        for i, angle in enumerate(angles):
            self.servo_sliders[i].set(angle)
            self.servo_labels[i].config(text=f"{self.servo_names[i]}: {angle}°")
        
        if self.controller.send_pose(angles):
            self.update_status("Pose sent", "green")
        else:
            self.update_status("Failed to send pose", "red")
    
    def send_gait(self, gait_name):
        """Send gait selection"""
        if not self.controller.connected:
            messagebox.showwarning("Warning", "Not connected to robot")
            return
        
        if self.controller.send_gait(gait_name):
            self.update_status(f"Gait: {gait_name}", "green")
        else:
            self.update_status("Failed to send gait", "red")
    
    def send_movement(self):
        """Send movement command"""
        if not self.controller.connected:
            messagebox.showwarning("Warning", "Not connected to robot")
            return
        
        forward = self.forward_var.get()
        turn = self.turn_var.get()
        
        if self.controller.send_move(forward, turn):
            self.update_status(f"Move: F={forward}, T={turn}", "green")
        else:
            self.update_status("Failed to send movement", "red")
    
    def update_speed(self, value):
        """Update speed multiplier"""
        speed = float(value)
        self.speed_label.config(text=f"{speed:.1f}x")
        
        if self.controller.connected:
            self.controller.send_command(f"SPEED:{speed:.1f}")
    
    def emergency_stop(self):
        """Emergency stop all movement"""
        if self.controller.connected:
            self.controller.emergency_stop()
            self.update_status("EMERGENCY STOP", "red")
        else:
            messagebox.showwarning("Warning", "Not connected to robot")
    
    def update_status(self, message, color="black"):
        """Update status bar"""
        status_text = f"Status: {message}"
        if self.controller.connected:
            status_text = f"Status: Connected - {message}"
        self.status_label.config(text=status_text, fg=color)
    
    def on_closing(self):
        """Handle window close event"""
        if self.controller.connected:
            self.controller.disconnect()
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()
