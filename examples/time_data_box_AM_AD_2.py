import matplotlib.pyplot as plt

# from pylab import plot, show, savefig, xlim, figure, \
                # hold, ylim, legend, boxplot, setp, axes
import csv

def setBoxColors(bp):
    plt.setp(bp['boxes'][0], color='blue')
    plt.setp(bp['caps'][0], color='blue')
    plt.setp(bp['caps'][1], color='blue')
    plt.setp(bp['whiskers'][0], color='blue')
    plt.setp(bp['whiskers'][1], color='blue')
    plt.setp(bp['fliers'][0], color='blue')
    plt.setp(bp['fliers'][1], color='blue')
    plt.setp(bp['medians'][0], color='blue')

    plt.setp(bp['boxes'][1], color='red')
    plt.setp(bp['caps'][2], color='red')
    plt.setp(bp['caps'][3], color='red')
    plt.setp(bp['whiskers'][2], color='red')
    plt.setp(bp['whiskers'][3], color='red')
    plt.setp(bp['fliers'][2], color='red')
    plt.setp(bp['fliers'][3], color='red')
    plt.setp(bp['medians'][1], color='red')

def define_box_properties(plot_name, color_code, label):
    for k, v in plot_name.items():
        plt.setp(plot_name.get(k), color=color_code)
         
    # use plot function to draw a small line to name the legend.
    plt.plot([], c=color_code, label=label)
    plt.legend()

with open('z_csv/time_data_FO_PINOCCHIO.csv', 'r') as file:
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

with open('z_csv/time_data_SO_PINOCCHIO.csv', 'r') as file2:
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

with open('z_csv/time_data_FOSO_AD.csv', 'r') as file3:
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
bp1 = plt.boxplot([Time_FO,Time_SO], positions=[1, 2], widths=0.6, showfliers=False)
# setBoxColors(bp)
bp2 = plt.boxplot([Time_FO2,Time_SO2], positions=[3, 4], widths=0.6, showfliers=False)
# setBoxColors(bp)

# setting colors for each groups
define_box_properties(bp1, '#D7191C', 'Analytical Method')
define_box_properties(bp2, '#2C7BB6', 'Automatic Differentiation')
# Set xtick labels
ax.set_xticklabels(['FO', 'SO','FO', 'SO'])
# Add gridlines
ax.grid(True)
# Set the y-axis to log scale
ax.set_yscale('log')
# Add labels and title
plt.title('ABA Partial Derivatives (gcc)')
plt.ylabel('Run Time [$\mu$s]')
# Show the plot
plt.show()

