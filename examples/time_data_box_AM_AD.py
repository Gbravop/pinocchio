import matplotlib.pyplot as plt

from pylab import plot, show, savefig, xlim, figure, \
                hold, ylim, legend, boxplot, setp, axes

import csv

def setBoxColors(bp):
    setp(bp['boxes'][0], color='blue')
    setp(bp['caps'][0], color='blue')
    setp(bp['caps'][1], color='blue')
    setp(bp['whiskers'][0], color='blue')
    setp(bp['whiskers'][1], color='blue')
    setp(bp['fliers'][0], color='blue')
    setp(bp['fliers'][1], color='blue')
    setp(bp['medians'][0], color='blue')

    setp(bp['boxes'][1], color='red')
    setp(bp['caps'][2], color='red')
    setp(bp['caps'][3], color='red')
    setp(bp['whiskers'][2], color='red')
    setp(bp['whiskers'][3], color='red')
    setp(bp['fliers'][2], color='red')
    setp(bp['fliers'][3], color='red')
    setp(bp['medians'][1], color='red')

with open('time_data_FO_PINOCCHIO.csv', 'r') as file:
    csv_reader = csv.reader(file)
    
    # Optionally, read header row
    header = next(csv_reader, None) 

    Time_FO = []
    # Time_SO = []

    for column in csv_reader:
        # Access individual columns by index
        Time_FO.append(float(column[0]))
        # Time_SO.append(float(column[1]))

    print(Time_FO[-1])
    # print(Time_SO[-1])
    # print(Time_SO[-1]+Time_FO[-1])

with open('time_data_SO_PINOCCHIO.csv', 'r') as file2:
    csv_reader2 = csv.reader(file2)
    
    # Optionally, read header row
    header2 = next(csv_reader2, None) 

    # Time_FO = []
    Time_SO = []

    for column in csv_reader2:
        # Access individual columns by index
        # Time_FO.append(float(column[0]))
        Time_SO.append(float(column[0]))

    # print(Time_FO[-1])
    print(Time_SO[-1])
    # print(Time_SO[-1]+Time_FO[-1])

with open('time_data_FOSO_AD.csv', 'r') as file3:
    csv_reader3 = csv.reader(file3)
    
    # Optionally, read header row
    header3 = next(csv_reader3, None) 

    Time_FO2 = []
    Time_SO2 = []

    for column in csv_reader3:
        # Access individual columns by index
        Time_FO2.append(float(column[0]))
        Time_SO2.append(float(column[0])+float(column[1]))

    print(Time_FO2[-1])
    print(Time_SO2[-1])
    # print(Time_SO2[-1]+Time_FO2[-1])

# Create the figure and axes
fig, ax = plt.subplots()
# Create the box plot
plt.boxplot([Time_FO,Time_SO,Time_FO2,Time_SO2], tick_labels=['FO', 'SO','FO2', 'SO2'], showfliers=False)
# Add gridlines
ax.grid(True)
# Set the y-axis to log scale
ax.set_yscale('log')
# Add labels and title
plt.title('ABA Partial Derivatives (gcc)')
plt.ylabel('Run Time [$\mu$s]')
# Show the plot
plt.show()

