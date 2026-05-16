# May 2026
# This file holds the code for the communications functions for the 
# UWT Project Herbinator's communications file

# Querying the esp for sensor state data periodically, and also manual functions
# Send updates to override existing behavior 
import requests
import asyncio
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox
# herby should host an http server that has JSON data dynamically updated to be sent over the network to the client
BASE_URL = "http://herbnet.local"  # DNS address
POLL_INTERVAL = 10


def send_command(endpoint, method="GET", data=None):
    url = f"{BASE_URL}/{endpoint}"

    try:
        if method == "GET":
            response = requests.get(url, timeout=1)
        elif method == "POST":
            response = requests.post(url, json=data, timeout=1)
        else:
            raise ValueError("Unsupported HTTP method")

        response.raise_for_status()
        try:
            return response.json()
        except ValueError:
            return response.text

    except requests.exceptions.RequestException as e:
        print(f"Error communicating with Herbinator: {e}")
        return None


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
def get_temperature():
    return send_command("Temperature")

def get_herbinator_state():
    return send_command("State")

def get_data_time():
    return send_command("Time")

def get_humidity():
    return send_command("Humidity")

def get_moisture():
    return send_command("Moisture")

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

        # Example label
        tk.Label(root, text="Temperature:").pack()
        tk.Label(root, textvariable=self.temperature_var).pack()

        # Log box
        self.log_box = tk.Text(root, height=10, width=50)
        self.log_box.pack()

        # Buttons
        tk.Button(root, text="Water Plant",
                  command=self.water_plant).pack()

        tk.Button(root, text="Resume",
                  command=self.resume_system).pack()

        # Start polling thread
        threading.Thread(
            target=self.start_polling_loop,
            daemon=True
        ).start()

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

    def refresh_data(self):
        self.log("Refreshing sensor data...")

        self.temperature_var.set(
            str(get_temperature() or "Disconnected")
        )
        self.humidity_var.set(
            str(get_humidity() or "Disconnected")
        )
        self.moisture_var.set(
            str(get_moisture() or "Disconnected")
        )
        self.state_var.set(
            str(get_herbinator_state() or "Disconnected")
        )
        self.time_var.set(
            str(get_data_time() or "Disconnected")
        )

        self.log("Sensor refresh complete.")

    async def polling_loop(self):
        while True:
            self.root.after(1, self.refresh_data)
            await asyncio.sleep(POLL_INTERVAL)

    def start_polling_loop(self):
        asyncio.run(self.polling_loop())


# ---------------- MAIN ---------------- #

def main():
    root = tk.Tk()
    app = Herbinator(root)
    root.mainloop()


if __name__ == "__main__":
    main()