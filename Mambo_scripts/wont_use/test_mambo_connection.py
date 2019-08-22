

from pyparrot.Minidrone import Mambo
import time

# you will need to change this to the address of YOUR mambo
# this is for Mambo_628236
mamboAddr = "D0:3A:93:36:E6:21"

# make my mambo object
# remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
mambo = Mambo(mamboAddr, use_wifi=False)

print("trying to connect")
success = mambo.connect(num_retries=5)
print("connected: %s" % success)

if (success):
    # get the state information
    print("sleeping 2 seconds")
    mambo.smart_sleep(2.0)
    print("2 seconds finished")
    mambo.ask_for_state_update()
    print("sleeping 5 seconds")
    mambo.smart_sleep(5.0)
    print("5 seconds finished")

    print("disconnect")
    mambo.disconnect()
