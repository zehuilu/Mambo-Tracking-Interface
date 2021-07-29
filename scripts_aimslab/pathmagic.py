#!/usr/bin/env python3
import os
import sys


class context:
    def __enter__(self):
        # assume the working directory is always the main directory
        sys.path.append(os.getcwd()+'/lib')

    def __exit__(self, *args):
        pass
