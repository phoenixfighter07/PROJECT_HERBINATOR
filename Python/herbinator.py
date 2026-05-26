# May 2026
# This file holds the code for the communications functions for the 
# UWT Project Herbinator's communications file

# Querying the esp for sensor state data periodically, and also manual functions
# Send updates to override existing behavior 
import requests
import time
import tkinter as tk
from tkinter import ttk, messagebox
import enum

# herby should host an http server that has JSON data dynamically updated to be sent over the network to the client
BASE_URL = "http://herbnet.local"  # DNS address
POLL_INTERVAL = 3

# The pause function is currently not implemented in the fsm logic, so pause never actually pauses anything (yet).
# same with manual watering functionality.
def send_command(endpoint, method="GET", data=None):
    url = f"{BASE_URL}/{endpoint}"

    try:
        if method == "GET":
            response = requests.get(url, timeout=3)
        elif method == "POST":
            response = requests.post(url, json=data, timeout=3)
        else:
            raise ValueError("Unsupported HTTP method")

        response.raise_for_status()
        return response.json()

    except requests.exceptions.RequestException as e:
        return {"error": str(e)}
    except ValueError as e:
        return {"error": f"Bad JSON: {e}"}

# display water level and manually water, water level reads as a percentage of total capacity :)
def herbinator_water():
    return send_command("water", method="POST")

# pause, can specify a time to be stopped, or stop indefinitely
# pauses the pump manually
def pause_herbinator(time=None):
    data = {"duration": time} if time else {}
    return send_command("pause", method="POST", data=data)


def unpause_herbinator():
    return send_command("resume", method="POST")

# read the temperature off the sensor, give a recommendation based off the temperature
# if state is off, or no connection, then commands should not process.
def get_device_data():
    return send_command("")

class Herbinator:

    def __init__(self, root):
        self.root = root
        self.root.title("Project Herbinator")

        # Variables
        self.temperature_var = tk.StringVar()
        self.humidity_var = tk.StringVar()
        self.moisture_var = tk.StringVar()
        self.state_var = tk.StringVar()
        self.time_var = tk.StringVar()

        # state and data labels
        tk.Label(root, text="Temperature:").pack()
        tk.Label(root, textvariable=self.temperature_var).pack()

        tk.Label(root, text="Humidity:").pack()
        tk.Label(root, textvariable=self.humidity_var).pack()

        tk.Label(root, text="Moisture:").pack()
        tk.Label(root, textvariable=self.moisture_var).pack()

        tk.Label(root, text="State:").pack()
        tk.Label(root, textvariable=self.state_var).pack()

        tk.Label(root, text="Last Update:").pack()
        tk.Label(root, textvariable=self.time_var).pack()
        # Log box
        self.log_box = tk.Text(root, height=10, width=50)
        self.log_box.pack()

        # BUTTTONZ :>
        tk.Button(root, text="Water Plant",
                  command=self.water_plant).pack()
        tk.Button(root, text="Pause",
                  command=self.pause_system).pack()
        tk.Button(root, text="Pause 1 Minute",
                command=self.pause_one_minute).pack()
        tk.Button(root, text="Resume",
                  command=self.resume_system).pack()
        
        # Start polling thread
        self.schedule_refresh()

    def schedule_refresh(self):
        self.refresh_data()
        self.root.after(POLL_INTERVAL * 1000, self.schedule_refresh)

    def log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.log_box.insert(
            tk.END,
            f"[{timestamp}] {message}\n"
        )
        self.log_box.see(tk.END)

    def water_plant(self):
        response = herbinator_water()
        self.log(f"Water command sent: {response}")

    def resume_system(self):
        response = unpause_herbinator()
        self.log(f"Resume command sent: {response}")

    def pause_system(self):
        response = pause_herbinator()
        self.log(f"Pause command sent: {response}")

    def pause_one_minute(self):
        response = pause_herbinator(60)
        self.log(f"Paused for 60s: {response}")

    def refresh_data(self):
        data = get_device_data()

        print(type(data))
        print(data)
        self.log(str(data))

        self.log("Refreshing sensor data...")
        data = get_device_data()

        if isinstance(data, dict):
            self.temperature_var.set(
                str(data.get("temperature", "Disconnected"))
            )
            self.humidity_var.set(
                str(data.get("humidity", "Disconnected"))
            )
            self.moisture_var.set(
                str(data.get("moisture", "Disconnected"))
            )
            self.state_var.set(
                str(data.get("state", "Disconnected"))
            )
            self.time_var.set(
                str(data.get("time", "Disconnected"))
            )
        else:
            self.temperature_var.set("Disconnected")
            self.humidity_var.set("Disconnected")
            self.moisture_var.set("Disconnected")
            self.state_var.set("Disconnected")
            self.time_var.set("Disconnected")
            self.log("Sensor refresh complete.")


# ---------------- MAIN ---------------- #

def main():
    root = tk.Tk()
    app = Herbinator(root)
    root.mainloop()


if __name__ == "__main__":
    main()