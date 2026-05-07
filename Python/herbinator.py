# May 2026
# This file holds the code for the communications functions for the 
# UWT Project Herbinator's communications file

# Querying the esp for sensor state data periodically, and also manual functions
# Send updates to override existing behavior 
import requests
# herby should host an http server that has JSON data dynamically updated to be sent over the network to the client
BASE_URL = "http://192.168.123.10"  # replace with herbinator's IP


def send_command(endpoint, method="GET", data=None):
    url = f"{BASE_URL}/{endpoint}"

    try:
        if method == "GET":
            response = requests.get(url)
        elif method == "POST":
            response = requests.post(url, json=data)
        else:
            raise ValueError("Unsupported HTTP method")

        response.raise_for_status()
        return response.json()  # assuming ESP returns JSON

    except requests.exceptions.RequestException as e:
        print(f"Error communicating with Herbinator: {e}")
        return None


# display water level and manually water, water level reads as a percentage of total capacity :)
def herbinator_water():
    send_command("water", method="POST")

# pause, can specify a time to be stopped, or stop indefinitely
# pauses the pump manually
def pause_herbinator(time=None):
    data = {"duration": time} if time else {}
    return send_command("pause", method="POST", data=data)


def unpause_herbinator():
    return send_command("resume", method="POST")

# read the temperature off the sensor, give a recommendation based off the temperature
def get_temperature():
    state = get_herbinator_state()
    if state:
        return state.get("Temperature")

def get_herbinator_state():
    return send_command("State")

def get_data_time():
    return send_command("Time")

def get_humidity():
    return send_command("Humidity")

def get_moisture():
    return send_command("Moisture")

def get_name():
    return send_command("Name")