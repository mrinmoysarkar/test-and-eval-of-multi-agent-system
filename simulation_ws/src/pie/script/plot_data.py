
from helper_functions import plot_data
from os import listdir
from os.path import isfile, join

import matplotlib.pyplot as plt

if __name__=="__main__":
    databasePath = "../flightData/plot_folder/"
    fileNames = [databasePath+f for f in listdir(databasePath) if isfile(join(databasePath, f))]
    plot_data(fileNames)
    plt.show()