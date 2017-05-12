#!/usr/bin/env python
"""
Auxiliary program for saving poses in spreadsheet and text file.
"""
# Standard libraries
import numpy as np
import sys
import time
#Excel read/write library
from openpyxl import load_workbook
from openpyxl import Workbook

def savedata(data, analyze=False):

    # Get the SP values from the user.
    time.sleep(0.2)
    sp_left = input("Introduce sLeft of test\n")
    sp_right = input("Introduce sRight of test\n")
    if analyze:
           data=analizedata(data) 
    header_text = np.array(['time', 'pos x', 'pos y', 'angle', 'difftime',
                            'diff pos x', 'diff pos y', 'diff angle'])
    full_data = np.vstack([header_text, data])
    # Name of the output file for the poses historic values.
    datestamp = "{}".format(time.strftime("%d_%m_%Y_%H%M"))
    filename = "datatemp/{}-L{}-R{}".format(datestamp, sp_left, sp_right)
    #TODO cocatenate the string on a more adequate way.
    filename_spreadsheet = filename + ".xlsx"
    data2spreadsheet(header_text, full_data, filename_spreadsheet)
    filename_txt = filename + ".txt"
    #TODO write the header better
    header_numpy = ("   time\t    pos x\t    pos y\t    angle\t difftime\t"
                   " diffposx\t diffposy\t diffangl")
    np.savetxt(filename_txt, data, delimiter='\t', fmt='%9.2f', 
               header=header_numpy)
#    data2textfile(header_text, data, filename_textfile)

def analizedata(data):
    
    rows = data.shape[0]
    cols = data.shape[1]            
    differential_data = np.zeros_like(data)
    for x in range(1, rows):
        for y in range(0, cols):
            differential_data[x,y] = data[x,y]-data[x-1,y]        
    data = np.hstack([data, differential_data])
    analized_data=data
    
    
    
    return analized_data

    
    
def data2spreadsheet(header, data, filename_spreadsheet):
    """
    Receives poses and time, and saves them in a spreadsheet.

    :param data: contains data to save in spreadsheet.
    :param filename_spreadsheet: name of spreadsheet where the data
     will be saved.
    """
    try:
        wb = load_workbook(filename_spreadsheet)
    except:
        wb = Workbook()
    ws = wb.active
    rows = data.shape[0]
    cols = data.shape[1]
    #Write in spreadsheet the headboard
    for y in range (0, cols):
        ws.cell(column=y+1, row=1, value=header[y])
    #Write in spreadsheet the data
    for x in range(1, rows):
        for y in range(0, cols):
            element = round(float(data[x,y]),2)
            ws.cell(column=y+1, row=x+1, value=element)
    wb.save(filename_spreadsheet)

#TODO NOT USED
def data2textfile(headboard, data, filename_textfile):
    """
    Receives poses and time, and saves them in a textfile.

    :param data: contains data to save in a textfile.
    :param filename_textfile: name of textfile where the data will
     be saved.
    """
    text = ''
    with open(filename_textfile, 'a') as outfile:
        rows = data.shape[0]
        cols = data.shape[1]
        #Write in spreadsheet the headboard
        for y in range (0, cols):
            text = text + "{}".format(headboard[y])
            add_space = True
            
        text = text + "\n"            
        for x in range(1, rows):
            for y in range(0, cols):
                element = float("{0:.2f}".format(data[x,y]))
                text = text + "{} \t\t\t".format(element)
            text = text + "\n"
        outfile.write(text)
