#!/usr/bin/env python
"""
Auxiliary program for reading, analyzing and writing data of poses and time.

This module allows:
-Read data of poses with their respective time, from a spreadsheet.
-Read data directly from an array of float32 Nx4.
-Analyze data by calculating differential values of position, time and lenght
 displaced and average speed of UGV
-Save data in spreadsheet and textfile.
-Save final data in master sheet.
"""
# Standard libraries
import math
import numpy as np
import sys
import time
#Excel read/write library
import openpyxl

def read_data(filename_spreadsheet="26_05_2017_1122-L0-R0.xlsx"):
    """
    It allows to read poses and time of spreadsheet to analyze or save them.

    :param filename_spreadsheet: name of spreadsheet that contain the data to
    be read.
    """
    try:
        wb = openpyxl.load_workbook(filename_spreadsheet)
    except IOError:
        wb = openpyxl.Workbook()
    ws = wb.active
    #Initialization of matrices for data.
    data = np.array([0, 0, 0, 0]).astype(np.float32)
    new_data = np.array([0, 0, 0, 0]).astype(np.float32)
    #The first row is the header, and the second is a set of zeros (already
    #initialized in the matrix). Begins to read row 3.
    row = 3
    #Number of columns in the matrix.
    cols = data.shape[0]
    #Loop for reading data.
    current_row_data = True
    while current_row_data:
        element = ws.cell(column=1, row=row).value
        if element == None:
            current_row_data = False
        else:
            for y in range (0, cols):
                element = ws.cell(column=y+1, row=row).value
                new_data[y] = element
            data = np.vstack([data, new_data])
            row +=1
    #Call to save and analyze data.
    save_data(data, analyze=True)
    return data

def save_data(data, analyze=False):
    """
    Receives poses and time of matrix to analyze and/or save them.

    :param data: Matrix of floats32 with data.
    :param analyze: Boolean that allows analyze or not the data.
    """
    #Get the SP values from the user.
    time.sleep(0.2)
    #Delete first row data (row of zeros)
    data = data[1:data.shape[0], :]
    #First sample, time zero.
    data[0:data.shape[0], 0] = data[0:data.shape[0], 0] - data[0, 0]
    #TODO Try, except correct value.
    sp_left = input("Introduce value of sp_left between 0 and 255\n")
    sp_right = input("Introduce value of sp_left between 0 and 255\n")
    #Call for data analysis function.
    if analyze:
        data, save_master = analyze_data(data)
        header_text = np.array(['Time', 'Pos x', 'Pos y', 'Angle', 'Diff Time',
                            'Diff Posx', 'Diff Posy', 'Diff Angl', 'Diff Long',
                            'Rel Speed', 'Rel AnSpd'])
    else:
        header_text = np.array(['Time', 'Pos x', 'Pos y', 'Angle'])
        save_master = False
    full_data = np.vstack([header_text, data])
    # Name of the output file for the poses historic values.
    datestamp = "{}".format(time.strftime("%d_%m_%Y_%H%M"))
    filename = "datatemp/{}-L{}-R{}".format(datestamp, sp_left, sp_right)
    name_spreadsheet = "{}.xlsx".format(filename)
    name_txt = "{}.txt".format(filename)
    name_mastertxt = "datatemp/masterfile.txt"
    #Header for numpy savetxt.
    header_numpy = ''
    cols = header_text.shape[0]
    for x in range (0, cols):
        element = header_text[x]
        element = '%9s' % (element)
        header_numpy = '{}{}\t'.format(header_numpy, element)
    #Call to save data in textfile.
    np.savetxt(name_txt, data, delimiter='\t', fmt='%9.3f',
               header=header_numpy, comments='')
#    np.savetxt(name_txt, data, delimiter='\t',
#               header=header_numpy, comments='')
    #Experiment conditions.
    exp_conditions = ("""-Use camera 2\n-Position initial experiment forward: 
                      right rear wheel profile (-1800, 400), rear axis UGV in 
                      axis y, in -1800 x\n-Position initial experiment backward: 
                      right front wheel profile (-600, 400), rear axis UGV in
                      axis y, in -600""")
                    
    #Call to save data in spreadsheet.
    data2spreadsheet(header_text, full_data, name_spreadsheet, exp_conditions)
    #Save data to masterfile.
    if save_master:
        #The average speed data is in the last row and last column.
        rows = data.shape[0]
        cols = data.shape[1]
        avg_speed = data[rows-1, cols-2]
        avg_ang_speed = data [rows-1, cols-1]
        data_master = np.array([avg_speed, avg_ang_speed, sp_left, sp_right,
                               datestamp])
        save2master_xlsx(data_master)
        save2master_txt(data_master)
#    data2textfile(header_text, data, filename_textfile)

def analyze_data(data):
    """
    Receives poses and time of matrix to analyze.

    Different time and position values are calculated between two data points.
    From these, the displaced length and the average speed of UGV are calculated.

    :param data: Matrix of floats32 with data.
    """
    rows = data.shape[0]
    cols = data.shape[1]
    #Differential data matrix: current data minus previous data.
    diff_data = np.zeros_like(data)
    #Vector differential length displaced.
    diff_length =  np.zeros(rows)
    #Vector differential speed.
    diff_speed = np.zeros(rows)
    #Vector differential angle speed.
    diff_angle_speed = np.zeros(rows)
    diff_data[1:rows, :] = data[1:rows, :] - data [0:(rows-1), :]
    diff_length[:] = pow(diff_data[:,1],2) + pow(diff_data[:,2],2)
    for x in range(1, rows):
        #Increase or decrease of displacement.
        if diff_data[x,1]>0:
            diff_length[x] = math.sqrt(diff_length[x])
        else:
            diff_length[x] = -math.sqrt(diff_length[x])
        #Prevents division between zero.
        if diff_data[x,0] != 0:
            #Speed in millimeters/second.
            diff_speed[x] = diff_length[x] / diff_data[x,0] * 1000
    diff_angle_speed[1:rows] = diff_data[1:rows, 3] / diff_data[1:rows, 0] * 1000
    #Complete data matrix with new data.
    diff_data = np.insert(diff_data, 4, diff_length, axis=1)
    diff_data = np.insert(diff_data, 5, diff_speed, axis=1)
    diff_data = np.insert(diff_data, 6, diff_angle_speed, axis=1)
    data = np.hstack([data, diff_data])
    #Vector sum of columns of data.
    sum_data = diff_data.sum(axis=0)
    a = np.array([0, 0, 0, 0])
    sum_data = np.hstack([a, sum_data])
    data = np.vstack([data, sum_data])
    #If you want to save to master file boolean True.
    save_master = True

    return data, save_master

def data2spreadsheet(header, data, filename_spreadsheet, exp_conditions):
    """
    Receives poses and time, and saves them in a spreadsheet.

    :param header: contains header to save in spreadsheet.
    :param data: contains data to save in spreadsheet.
    :param filename_spreadsheet: name of spreadsheet where the data
     will be saved.
    """
    try:
        wb = openpyxl.load_workbook(filename_spreadsheet)
    except:
        wb = openpyxl.Workbook()
    ws = wb.active
    rows = data.shape[0]
    cols = data.shape[1]
    #Write in spreadsheet the headboard
    for y in range (0, cols):
        ws.cell(column=y+1, row=1, value=header[y])
    #Write in spreadsheet the data
    for x in range(1, rows):
        for y in range(0, cols):
            element = float(data[x,y])
            ws.cell(column=y+1, row=x+1, value=element).number_format = '0.00'
            my_cell = ws.cell(column=y+1, row=x+1)
            ws.column_dimensions[my_cell.column].width = 10
    ws.merge_cells('M1:W3')
    ws.cell('M1').value = exp_conditions
    #Statistics
    ws.cell('M1').value = exp_conditions
    ws.merge_cells('M5:O5')
    ws.cell('M5').value = "Average Differential Time:"
    ws.merge_cells('M6:O6')
    ws.cell('M6').value = "Median Differential Time:"
    ws.merge_cells('M7:O7')
    ws.cell('M7').value = "Mode Differential Time:"
    ws.merge_cells('M8:O8')
    ws.cell('M8').value = "Variance Differential Time:"
    ws.merge_cells('M9:O9')
    ws.cell('M9').value = "Tipical Deviation Differential Time:"
    ws.merge_cells('M10:O10')
    ws.cell('M10').value = "Average Differential Pos x:"
    ws.merge_cells('M11:O11')
    ws.cell('M11').value = "Median Differential Pos x:"
    ws.merge_cells('M12:O12')
    ws.cell('M12').value = "Mode Differential Pos x:"
    ws.merge_cells('M13:O13')
    ws.cell('M13').value = "Variance Differential Pos x:"
    ws.merge_cells('M14:O14')
    ws.cell('M14').value = "Tipical Deviation Differential Pos x:"
    
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
                element = float("{0:.3f}".format(data[x,y]))
                text = text + "{} \t\t\t".format(element)
            text = text + "\n"
        outfile.write(text)

def save2master_xlsx(data_master):
    """
    Average speed, setpoint left and right wheels are saved in the same spreadsheet.

    :param data_master: array that contains the average lineal speed, the
    average angular speed, the set point left, the set point right, and the
    name of datafile.
    """
    try:
        wb = openpyxl.load_workbook("datatemp/masterfile.xlsx")
    except:
        wb = openpyxl.Workbook()
    ws = wb.active
    #Next empty row search.
    row = 1
    written_row = True
    while written_row:
        element = ws.cell(column=1, row=row).value
        if element == None:
            written_row = False
        else:
            row +=1
    #Write data in empty row.
    cols = data_master.shape[0]
    for y in range (0, cols):
        if y < 2:
            element = float(data_master[y])
        else:
            element = data_master[y]
        ws.cell(column=y+1, row=row, value=element).number_format = '0.00'
        my_cell = ws.cell(column=y+1, row=row)
        if y < 3:
            ws.column_dimensions[my_cell.column].width = 10
        else:
            ws.column_dimensions[my_cell.column].width = 15
    wb.save("datatemp/masterfile.xlsx")

def save2master_txt(data_master):
    """
    Average speed, setpoint left and right wheels are saved in the same textfile.

    :param data_master: array that contains the average lineal speed, the
    average angular speed, the set point left, the set point right, and the
    name of datafile.
    """
    text = ''
    with open("datatemp/masterfile.txt", 'a') as outfile:
    #TODO improve format
        text = ''
        cols = data_master.shape[0]
        for y in range (0, cols):
            element = data_master[y]
            if y < 2:
                element = float(data_master[y])
                element = '%9.3f' % (element)
            else:
                element = '%9s' % (element)
            text = '{}{}\t'.format(text, element)
        text = '{}\n'.format(text)
        outfile.write(text)
