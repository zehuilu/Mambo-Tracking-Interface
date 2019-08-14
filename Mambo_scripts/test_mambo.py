

from pyparrot.Minidrone import Mambo
import time

# you will need to change this to the address of YOUR mambo
# this is for Mambo_628236
mamboAddr = "D0:3A:93:36:E6:21"

# make my mambo object
# remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
mambo = Mambo(mamboAddr, use_wifi=False)

print("trying to connect")
success = mambo.connect(num_retries=3)
print("connected: %s" % success)

if (success):
    # get the state information
    print("sleeping")
    mambo.smart_sleep(2)
    mambo.ask_for_state_update()
    mambo.smart_sleep(2)

    print("taking off!")
    mambo.safe_takeoff(5)



    idx = 0
    while idx < 80:
        t0 = time.time()
        mambo.fly_direct(roll=0, pitch=50*(-1)**idx, yaw=0, vertical_movement=0, duration=0.05)
        t1 = time.time()
        dt = t1 - t0
        print("time interval for fly command")
        print(dt)
        idx += 1



    


    print("landing")
    mambo.safe_land(5)

    print("disconnect")
    mambo.disconnect()
