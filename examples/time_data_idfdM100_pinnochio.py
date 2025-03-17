import matplotlib.pyplot as plt
import numpy as np
import csv

N_list = [1,2,3,4,5,6,7,8,10,13,16,24,29,36,44,54,66,81,100]

with open('time_data_FO_PINOCCHIO_100.csv', 'r') as file:
    csv_reader = csv.reader(file)
    csv_data = list(csv_reader) 

    MTime_FO = []

    val = 0.0
    for j in range(0,len(csv_data[0])):
        for i in range(0,len(csv_data)):
            val = val + float(csv_data[i][j])
        MTime_FO.append(val/len(csv_data))
        val = 0.0

with open('time_data_SO_PINOCCHIO_100.csv', 'r') as file2:
    csv_reader2 = csv.reader(file2)
    csv_data2 = list(csv_reader2) 

    MTime_SO = []

    val2 = 0.0
    for j in range(0,len(csv_data2[0])):
        for i in range(0,len(csv_data2)):
            val2 = val2 + float(csv_data2[i][j])
        MTime_SO.append(val2/len(csv_data2))
        val2 = 0.0
    
with open('time_data_FOid_PINOCCHIO_100.csv', 'r') as file1:
    csv_reader1 = csv.reader(file1)
    csv_data1 = list(csv_reader1) 

    MTime_FO1 = []

    val1 = 0.0
    for j in range(0,len(csv_data1[0])):
        for i in range(0,len(csv_data1)):
            val1 = val1 + float(csv_data1[i][j])
        MTime_FO1.append(val1/len(csv_data1))
        val1 = 0.0
    
with open('time_data_SOid_PINOCCHIO_100.csv', 'r') as file3:
    csv_reader3 = csv.reader(file3)
    csv_data3 = list(csv_reader3) 

    MTime_SO3 = []

    val3 = 0.0
    for j in range(0,len(csv_data3[0])):
        for i in range(0,len(csv_data3)):
            val3 = val3 + float(csv_data3[i][j])
        MTime_SO3.append(val3/len(csv_data3))
        val3 = 0.0
     
# Create the figure and axes
fig, ax = plt.subplots()
# Create the box plot
plt.plot(N_list,MTime_FO, label='FO_ABA')
plt.plot(N_list,MTime_SO, label='SO_ABA')
plt.plot(N_list,MTime_FO1, label='FO_RNEA')
plt.plot(N_list,MTime_SO3, label='SO_RNEA')

# Set the xy-axis to log scale
ax.set_xscale('log')
ax.set_yscale('log')

# Add gridlines
ax.minorticks_on()
# Customize grids
ax.grid(which='major', linestyle='-', linewidth='0.5', color='gray')
ax.grid(which='minor', linestyle=':', linewidth='0.25', color='gray')

# Add labels and title
plt.title('Dynamics Partial Derivatives - Analytical Method (gcc)')
plt.xlabel('Number of Links N [-]')
plt.ylabel('Run Time [$\mu$s]')
plt.legend()
# Show the plot
plt.show()

