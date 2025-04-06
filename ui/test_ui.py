import tkinter as tk
from tkinter import ttk
import socket
import threading

UDP_IP = "192.168.106.235"
UDP_PORT = 123410


class RobotController:
    def __init__(self, master):
        self.master = master
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running = True

        master.title("Robot Controller")
        self.create_widgets()
        self.start_listener()

    def create_widgets(self):
        # Управление скоростью
        self.speed_frame = ttk.LabelFrame(self.master, text="Velocity Control")
        self.speed_frame.grid(row=0, column=0, padx=10, pady=10)

        self.lin_vel = tk.Scale(self.speed_frame, from_=-500, to=500, orient=tk.HORIZONTAL,
                                label="Linear Velocity (mm/s)")
        self.lin_vel.pack(padx=5, pady=5)

        self.ang_vel = tk.Scale(self.speed_frame, from_=-3.14, to=3.14, resolution=0.1, orient=tk.HORIZONTAL,
                                label="Angular Velocity (rad/s)")
        self.ang_vel.pack(padx=5, pady=5)

        self.btn_send = ttk.Button(self.speed_frame, text="Send Velocity", command=self.send_velocity)
        self.btn_send.pack(pady=5)

        # PID настройки
        self.pid_frame = ttk.LabelFrame(self.speed_frame, text="PID Settings")
        self.pid_frame.pack(fill=tk.X, padx=5, pady=5)

        self.kp = ttk.Entry(self.pid_frame, width=8)
        self.ki = ttk.Entry(self.pid_frame, width=8)
        self.kd = ttk.Entry(self.pid_frame, width=8)
        self.kff = ttk.Entry(self.pid_frame, width=8)

        ttk.Label(self.pid_frame, text="Kp:").grid(row=0, column=0)
        self.kp.grid(row=0, column=1)
        ttk.Label(self.pid_frame, text="Ki:").grid(row=0, column=2)
        self.ki.grid(row=0, column=3)
        ttk.Label(self.pid_frame, text="Kd:").grid(row=0, column=4)
        self.kd.grid(row=0, column=5)
        ttk.Label(self.pid_frame, text="Kff:").grid(row=0, column=6)
        self.kff.grid(row=0, column=7)

        self.btn_pid = ttk.Button(self.pid_frame, text="Set PID", command=self.send_pid)
        self.btn_pid.grid(row=1, column=0, columnspan=8, pady=5)

        # Статус
        self.status = ttk.Label(self.master, text="Disconnected", foreground="red")
        self.status.grid(row=1, column=0)

    def send_velocity(self):
        lin = self.lin_vel.get()
        ang = self.ang_vel.get()
        cmd = f"SET_ROBOT_VELOCITY {lin} {ang}"
        self.sock.sendto(cmd.encode(), (UDP_IP, UDP_PORT))

    def send_pid(self):
        cmd = f"SET_COEFF {self.kp.get()} {self.ki.get()} {self.kd.get()} {self.kff.get()}"
        self.sock.sendto(cmd.encode(), (UDP_IP, UDP_PORT))

    def start_listener(self):
        def listener():
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("0.0.0.0", UDP_PORT))
            while self.running:
                try:
                    data, addr = sock.recvfrom(1024)
                    self.status.config(text=data.decode(), foreground="green")
                except:
                    pass

        threading.Thread(target=listener, daemon=True).start()

    def on_close(self):
        self.running = False
        self.master.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    controller = RobotController(root)
    root.protocol("WM_DELETE_WINDOW", controller.on_close)
    root.mainloop()