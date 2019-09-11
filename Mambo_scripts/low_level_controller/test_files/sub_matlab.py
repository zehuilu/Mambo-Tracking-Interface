import csv
import numpy as np
import glob
import os

if __name__ == "__main__":
    Directory = '/home/roahmlab/Mambo_scripts/combo_old/traj_lib/'
    BaseName = 'traj_'

    idx = 0
    while idx < 5:
        #FileName = Directory + BaseName + str(idx) + '.csv'
        csv_file_list = glob.glob("/home/roahmlab/Mambo_scripts/combo_old/traj_lib/*.csv")
        csv_file_list = sorted(csv_file_list, key=os.path.getmtime)
        #FileName = csv_file_list[-1]
        print(csv_file_list)
        #print(FileName)

        #data = np.genfromtxt(FileName, dtype=float, delimiter=',')

        print(idx)
        #print(data)

        

        idx += 1