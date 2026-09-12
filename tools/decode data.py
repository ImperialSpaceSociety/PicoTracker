# Created by Medad Newman 6/8/19
# This program takes the byte data from no_pips_data.txt which is the raw data output from DL-fldigi and prints out all
# the strings that pass the checksum.

import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

POLYNOMIAL = 0x1021
PRESET = 0xFFFF
def _initial(c):
    crc = 0
    c = c << 8
    for j in range(8):
        if (crc ^ c) & 0x8000:
            crc = (crc << 1) ^ POLYNOMIAL
        else:
            crc = crc << 1
        c = c << 1
    return crc

_tab = [ _initial(i) for i in range(256) ]

def _update_crc(crc, c):
    cc = 0xff & c

    tmp = (crc >> 8) ^ cc
    crc = (crc << 8) ^ _tab[tmp & 0xff]
    crc = crc & 0xffff
    #print (crc)

    return crc

def crc(str):
    crc = PRESET
    for c in str:
        crc = _update_crc(crc, ord(c))
    return crc

def crcb(*i):
    crc = PRESET
    for c in i:
        crc = _update_crc(crc, c)
    return crc

#f = open("no_pips_data.txt", "r")
def bytes_from_file(filename, chunksize=1):
    with open(filename, "rb") as f:
        while True:
            chunk = f.read(chunksize)
            if chunk:
                for b in chunk:
                    yield b
            else:
                break


def analyse_data(file_path):
    data_list = []

    a1 = None # $
    a2 = None # $
    a3 = None # non $

    data = ""
    checksum = ""

    adding_data = False
    adding_checksum = False
    checksum_counter = 0

    for b in bytes_from_file(file_path):

        if checksum_counter == 4:
            checksum_counter = 0
            adding_checksum = False

            # now verify checksum
            calculated_checksum = hex(crc(data))[2:].upper()
            if calculated_checksum == checksum:
                print(data)
                data_list.append(data.split(','))
            # now reset both checksum and data
            data = ""
            checksum = ""

        if adding_checksum:
            checksum+= chr(b)
            checksum_counter +=1

        if adding_data and chr(b) == "*":
            adding_data = False
            adding_checksum = True

        if adding_data:
            data+=chr(b)


        # the moving up
        a1 = a2
        a2 = a3
        a3 = b

        # Now check if a1 a2 a3 are correct
        if a1 == 36 and a2 == 36 and a3 != 36:
            adding_data = True
            data+=chr(a3)

    return data_list

def get_deltas(data_list):
    df = pd.DataFrame(np.array(data_list),
                      columns =(['Callsign', 'str', 'time','lat','long','alt','sats','batt','opbyte','temp']))
    df['time'] = pd.to_datetime(df['time'], format='%H%M%S')
    df['delta'] = (df['time'] - df['time'].shift()).fillna(0)
    print(df)
    print(df.describe())
    print(df.count())

    min_delta = df['delta'].loc[1:].astype('timedelta64[s]').min()
    max_delta = df['delta'].astype('timedelta64[s]').max()
    return df,min_delta,max_delta


if __name__ == "__main__":

    fig, ax = plt.subplots()
    fig.suptitle("Distribution of duration for the whole acquire gps fix and transmit cycle.\n"
                 "This graph compares both using pips and \n not using pips. n = 154")

    file_paths = ['with_pips_data.txt',"no_pips_data.txt"]
    data_dfs = []
    for file_path in file_paths:
        data_list =analyse_data(file_path)
        df,min_delta,max_delta = get_deltas(data_list)
        data_dfs.append(df['delta'].astype('timedelta64[s]'))
        counts, bins, patches = ax.hist(df['delta'].astype('timedelta64[s]'),
                                       bins = range(int(min_delta),int(max_delta)+1),
                                        label=file_path,
                                        alpha=0.5)

    #plt.hist(data_dfs, label=['With pips', 'Without Pips'])
    ax.legend()
    ax.set_xlabel("cycle duration(s)")
    ax.set_ylabel("Frequency")
    #ax.locator_params(nbins=10, axis='x')
    #max_xticks = 25
    #xloc = plt.MaxNLocator(max_xticks)
    #ax.xaxis.set_major_locator(xloc)
    # Set the ticks to be at the edges of the bins.
    #ax.set_xticks(bins)

    plt.show()