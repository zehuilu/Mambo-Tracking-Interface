#!/usr/bin/env python3
import os
import pathmagic
with pathmagic.context():
    from MamboControllerInterface import MamboControllerInterface


if __name__ == "__main__":
    # load the configuration as a dictionary

    # # for Real-time-Task-Allocation-and-Path-Planning
    # config_file_name = os.path.expanduser("~") + \
    #     "/Real-time-Task-Allocation-and-Path-Planning/experiment/config_aimslab_ex.json"

    # for learning from directional correction
    config_file_name = os.path.expanduser("~") + \
        "/Learning-from-Directional-Corrections/experiments/config_aimslab.json"

    mocap_string = "QUALISYS"

    Controller = MamboControllerInterface(config_file_name, mocap_string)
    Controller.run_controller()
