#!/usr/bin/env python3
import os
import sys


class context:
    def __init__(self, EXTERNAL_FLAG=False):
        # True if use external libraries
        self.external_flag = EXTERNAL_FLAG

    def __enter__(self):
        # assume the working directory is always the main directory
        sys.path.append(os.getcwd()+'/lib')
        
        # True if use external libraries
        if self.external_flag:
            if sys.platform == "linux":
                sys.path.append('/home/aims-zehui/Real-time-Task-Allocation-and-Path-Planning/build')
                sys.path.append('/home/aims-zehui/Real-time-Task-Allocation-and-Path-Planning/src')
            elif sys.platform == "darwin":
                sys.path.append('/Users/zehui/Real-time-Task-Allocation-and-Path-Planning/build')
                sys.path.append('/Users/zehui/Real-time-Task-Allocation-and-Path-Planning/src')
            else:
                pass

    def __exit__(self, *args):
        pass
