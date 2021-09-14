#!/usr/bin/env python3
import os
import sys
import pathmagic
with pathmagic.context():
    from MamboControllerInterface import MamboControllerInterface


if __name__ == "__main__":
    # load mambo index from command line arguments
    if len(sys.argv) == 2:
        mambo_idx = sys.argv[0]
    else:
        mambo_idx = 1

    # load the configuration as a dictionary

    # for Real-time-Task-Allocation-and-Path-Planning
    config_file_name = os.path.expanduser("~") + \
        "/Real-time-Task-Allocation-and-Path-Planning/experiment/config_aimslab_ex_" + str(mambo_idx) + ".json"

    # # for learning from directional correction
    # config_file_name = os.path.expanduser("~") + \
    #     "/Learning-from-Directional-Corrections/experiments/config_aimslab.json"

    mocap_string = "QUALISYS"

    Controller = MamboControllerInterface(config_file_name, mocap_string)
    Controller.run_controller()
