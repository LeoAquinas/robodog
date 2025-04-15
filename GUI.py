import customtkinter as ctk
import subprocess
import webbrowser
import os
import signal
import platform

ctk.set_appearance_mode("dark")  # Options: "dark", "light", "system"
ctk.set_default_color_theme("blue")  # Default theme is blue

IS_WINDOWS = platform.system() == "Windows"

class MyApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("GDP Assignment")
        self.geometry("800x500")

        self.running_processes = {}
        self.bring_up_running = False
        self.started = False

        self.title_label = ctk.CTkLabel(self, text="Hello from Bottom Gears", font=("Arial", 30, "bold"))
        self.title_label.pack(pady=20, anchor="center")

        self.bringup_frame = ctk.CTkFrame(self)
        self.bringup_frame.pack(padx=20, pady=10, fill="x", expand=False)

        self.middle_frame = ctk.CTkFrame(self)
        self.middle_frame.pack(padx=20, pady=10, fill="both", expand=True)

        self.maps_frame = ctk.CTkFrame(self.middle_frame)
        self.maps_frame.pack(side="left", padx=10, pady=10, fill="both", expand=True)

        self.foxglove_frame = ctk.CTkFrame(self.middle_frame)
        self.foxglove_frame.pack(side="right", padx=10, pady=10, fill="both", expand=True)

        self.bringup_frame.grid_columnconfigure(0, weight=1)
        self.bringup_frame.grid_columnconfigure(1, weight=1)
        self.maps_frame.grid_columnconfigure(0, weight=1)
        self.maps_frame.grid_columnconfigure(1, weight=1)
        self.foxglove_frame.grid_columnconfigure(0, weight=1)

        self.create_bringup_widgets()
        self.create_maps_widgets()
        self.create_foxglove_widgets()

        self.bringup_buttons = {}
        self.maps_buttons = {}
        self.foxglove_buttons = {}

        self.collect_buttons()
        self.update_button_states()

    def create_bringup_widgets(self):
        self.bringup_label = ctk.CTkLabel(self.bringup_frame, text="Bring Up Control", font=("Arial", 18, "bold"))
        self.bringup_label.grid(row=0, column=0, columnspan=2, pady=(10,5), sticky="nsew")

        self.btn_bringup = ctk.CTkButton(self.bringup_frame, text="Bring Up", command=lambda: self.on_button_click("Bring Up"))
        self.btn_bringup.grid(row=1, column=0, columnspan=2, padx=10, pady=10, sticky="nsew")

        self.btn_start = ctk.CTkButton(self.bringup_frame, text="Start", command=lambda: self.on_button_click("Start"))
        self.btn_start.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")

        self.btn_end = ctk.CTkButton(self.bringup_frame, text="End", command=lambda: self.on_button_click("End"))
        self.btn_end.grid(row=2, column=1, padx=10, pady=10, sticky="nsew")

    def create_maps_widgets(self):
        self.maps_label = ctk.CTkLabel(self.maps_frame, text="Maps", font=("Arial", 18, "bold"))
        self.maps_label.grid(row=0, column=0, columnspan=2, pady=(10,5), sticky="nsew")

        self.btn_2d = ctk.CTkButton(self.maps_frame, text="2D Mapping", command=lambda: self.on_button_click("2D Mapping"))
        self.btn_2d.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        self.btn_3d = ctk.CTkButton(self.maps_frame, text="3D Mapping", command=lambda: self.on_button_click("3D Mapping"))
        self.btn_3d.grid(row=1, column=1, padx=10, pady=10, sticky="nsew")

        self.btn_save_map = ctk.CTkButton(self.maps_frame, text="Save Map", command=lambda: self.on_button_click("Save Map"))
        self.btn_save_map.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")

        self.btn_open_map = ctk.CTkButton(self.maps_frame, text="Open Map", command=lambda: self.on_button_click("Open Map"))
        self.btn_open_map.grid(row=2, column=1, padx=10, pady=10, sticky="nsew")

        self.btn_stop_all_maps = ctk.CTkButton(self.maps_frame, text="✖", width=10, command=self.stop_all_maps)
        self.btn_stop_all_maps.grid(row=0, column=1, sticky="ne", padx=5, pady=5)

    def create_foxglove_widgets(self):
        self.foxglove_label = ctk.CTkLabel(self.foxglove_frame, text="Foxglove", font=("Arial", 18, "bold"))
        self.foxglove_label.grid(row=0, column=0, pady=(10,5), sticky="nsew")

        self.btn_foxglove = ctk.CTkButton(self.foxglove_frame, text="Open Foxglove", command=lambda: self.on_button_click("Open Foxglove"))
        self.btn_foxglove.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

    def collect_buttons(self):
        self.bringup_buttons["Bring Up"] = self.btn_bringup
        self.bringup_buttons["Start"] = self.btn_start
        self.bringup_buttons["End"] = self.btn_end

        self.maps_buttons["2D Mapping"] = self.btn_2d
        self.maps_buttons["3D Mapping"] = self.btn_3d
        self.maps_buttons["Save Map"] = self.btn_save_map
        self.maps_buttons["Open Map"] = self.btn_open_map

        self.foxglove_buttons["Open Foxglove"] = self.btn_foxglove

    def update_button_states(self):
        self.bringup_buttons["Bring Up"].configure(state="normal")
        self.bringup_buttons["Start"].configure(state="normal" if self.bring_up_running and not self.started else "disabled")
        self.bringup_buttons["End"].configure(state="normal" if self.started else "disabled")

        for button in self.maps_buttons.values():
            button.configure(state="normal" if self.started else "disabled")
        for button in self.foxglove_buttons.values():
            button.configure(state="normal" if self.started else "disabled")

    def stop_process(self, button_text):
        if button_text in self.running_processes:
            process = self.running_processes[button_text]
            try:
                if IS_WINDOWS:
                    process.send_signal(signal.CTRL_BREAK_EVENT)
                else:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait()
                del self.running_processes[button_text]
                print(f"Stopped {button_text} successfully.")
            except Exception as e:
                print(f"Failed to stop {button_text}: {e}")
        else:
            print(f"No running process for {button_text}.")

    def stop_all_maps(self):
        for key in ["2D Mapping", "3D Mapping", "Save Map", "Open Map"]:
            self.stop_process(key)

    def on_button_click(self, button_text):
        print(f"{button_text} clicked!")

        commands = {
            "Start": "ros2 launch opendog_bringup launch_robot.launch.py",
            "2D Mapping": "ros2 launch opendog_bringup grid_mapping.launch.py",
            "3D Mapping": "ros2 launch opendog_bringup full_mapping.launch.py",
            "Save Map": "ros2 launch opendog_bringup save_big_map.launch.py",
            "Open Map": "ros2 launch opendog_bringup open_map.launch.py"
        }

        if button_text == "Open Foxglove":
            ws_url = "ws://192.168.77.154:8765"
            url = f"https://studio.foxglove.dev/?ws={ws_url}"
            try:
                firefox = webbrowser.get("firefox")
                firefox.open(url)
            except webbrowser.Error:
                # Fallback to the default browser if Firefox is not found.
                webbrowser.open(url)
            return

        if button_text == "Bring Up":
            self.bring_up_running = not self.bring_up_running
            self.bringup_buttons["Bring Up"].configure(fg_color="green" if self.bring_up_running else "blue")
            if not self.bring_up_running:
                self.started = False
            self.update_button_states()
            return

        if button_text == "Start":
            if not self.bring_up_running:
                print("Please toggle Bring Up first!")
                return
            if "Start" in self.running_processes:
                print("Process already running.")
                return
            try:
                creationflags = subprocess.CREATE_NEW_PROCESS_GROUP if IS_WINDOWS else 0
                process = subprocess.Popen(commands["Start"], shell=True, creationflags=creationflags)
                self.running_processes["Start"] = process
                self.started = True
                print("Bring Up started.")
            except Exception as e:
                print(f"Failed to start Bring Up: {e}")
            self.update_button_states()
            return

        if button_text == "End":
            self.stop_process("Start")
            self.started = False
            self.update_button_states()
            return

        if not self.started:
            print("You must press 'Start' first!")
            return

        # For Save Map, check if 2D Mapping is running and choose command accordingly.
        if button_text == "Save Map":
            # Check if 2D Mapping process is active.
            if "2D Mapping" in self.running_processes:
                # If 2D Mapping is running, use alternative command (e.g., using SLAM Toolbox).
                command = "ros2 launch opendog_bringup save_small_map.launch.py"
            else:
                # Otherwise, use the default command.
                command = commands["Save Map"]
        else:
            command = commands.get(button_text)

        # command = commands.get(button_text)
        if command:
            if button_text in self.running_processes:
                print(f"{button_text} is already running.")
                return
            try:
                creationflags = subprocess.CREATE_NEW_PROCESS_GROUP if IS_WINDOWS else 0
                process = subprocess.Popen(command, shell=True, creationflags=creationflags)
                self.running_processes[button_text] = process
                print(f"Started {button_text}: {command}")
            except Exception as e:
                print(f"Failed to start {button_text}: {e}")
        else:
            print(f"No command found for {button_text}.")

if __name__ == "__main__":
    app = MyApp()
    app.mainloop()
