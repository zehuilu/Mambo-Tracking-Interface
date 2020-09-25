#!/usr/bin/env python3
import json

# Read the configuration from the json file
config_file_name = "config.json"
json_file = open(config_file_name)
config_data = json.load(json_file)


a = config_data["QUALISYS"]["IP_MOCAP_SERVER"]

b = config_data['MAMBO']["BLUETOOTH_ADDRESS"]

c = int(config_data['PHASESPACE']["FREQUENCY_MOCAP"])

d = float(config_data['LOW_LEVEL_CONTROLLER']["KD_ROLL"])


print(a)
print(b)
print(c)
print(d)