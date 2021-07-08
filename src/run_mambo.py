#!/usr/bin/env python3
with pathmagic.context():
    from MamboControllerInterface import MamboControllerInterface


if __name__ == "__main__":
    config_file_name = "config_aimslab.json"
    mocap_string = "QUALISYS"

    Controller = MamboControllerInterface(config_file_name, mocap_string)
    Controller.run_LLC()
