import matplotlib.pyplot as plt
import csv

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

# Create the figure and axes
fig, ax = plt.subplots()
# Create the box plot
plt.boxplot([Time_FO,Time_SO], tick_labels=['FO', 'SO'], showfliers=False)
# Add gridlines
ax.grid(True)
# Set the y-axis to log scale
ax.set_yscale('log')
# Add labels and title
plt.title('ABA Partial Derivatives - AM (gcc)')
plt.ylabel('Run Time [$\mu$s]')
# Show the plot
plt.show()

