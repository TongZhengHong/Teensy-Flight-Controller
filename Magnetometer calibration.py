import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import numpy as np
import math

original = [[], [], []]
temp = [[], [], []]
dest = [[], [], []]
outlier = [[], [], []]

mag_scale = []
mag_offset = []

line_x = [[-600, 600], [0, 0], [0, 0]]
line_y = [[0, 0], [-600, 600], [0, 0]]
line_z = [[0, 0], [0, 0], [-600, 600]]

FILE_PATH = "Users/zhenghong/Documents/UAV Project 2018/Teensy-Flight-Controller/"
INPUT_FILE = "mag_output.txt"
file = open(INPUT_FILE, "r")

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

def read_data():
    for line in file:
        line = line.strip()
        mag_x, mag_y, mag_z = line.split(",")
        
        original[0].append(float(mag_x))
        original[1].append(float(mag_y))
        original[2].append(float(mag_z))

        temp[0].append(float(mag_x))
        temp[1].append(float(mag_y))
        temp[2].append(float(mag_z))

        dest[0].append(float(mag_x))
        dest[1].append(float(mag_y))
        dest[2].append(float(mag_z))

        outlier[0].append(float(mag_x))
        outlier[1].append(float(mag_y))
        outlier[2].append(float(mag_z))

def find_average():
    average = [0] * 3
    
    #Compute the average of all raw values --> Should give the average center of each axis
    for i in range(3):
        for j in range(len(dest[i])):
            average[i] += dest[i][j]
        average[i] /= len(dest[i])

    for i in range(len(dest[0])):
        for j in range(3):
            dest[j][i] = (dest[j][i] - average[j])

def compare_distances():
    #Compute the average of each axis
    THRESHOLD = 40
    outlier_index = []

    for i in range(len(temp[0])):                   #Every coordinate
        smallest_dist = [9999] * 6
        if i%500 == 0:
            print(i)
            
        for j in range(len(temp[0])):               #Compare with any other coordinate
            x = temp[0][j] - temp[0][i]
            y = temp[1][j] - temp[1][i]
            z = temp[2][j] - temp[2][i]
            
            distance = math.sqrt(math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2))
            if distance != 0:
                smallest_dist[5] = distance
                smallest_dist.sort()
            #print(smallest_dist, j)

        ave_dist = 0.0
        for k in range(5):
            ave_dist += smallest_dist[k]
        ave_dist /= 5
        #print(ave_dist)

        if ave_dist > THRESHOLD:
            print(ave_dist)
            outlier_index.append(i)

    outlier_index.sort(reverse=True)
    print(outlier_index)
    for index in outlier_index:
        temp[0].pop(index)
        temp[1].pop(index)
        temp[2].pop(index)

    mag_scales = compute_scales(temp)
    mag_offset = compute_offsets(temp)

    #Apply scales and offsets to original data
    for i in range(len(temp[0])):
        for j in range(3):
            temp[j][i] = (temp[j][i] - mag_offset[j]) * mag_scales[j]

def compute_offsets(points):
    mag_offset = []
    data = [[], [], []]
    
    print("Offsets: ")
    for i in range(3):
        data[i] = sorted(points[i])
        final = len(data[i])-1
        
        mag_offset.append((data[i][final] + data[i][0])/2)
        print(mag_offset[i], end=" ")
    print()
    
    return mag_offset

def compute_scales(points):
    mag_scale = []
    data = [[], [], []]
    
    for i in range(3):
        data[i] = sorted(points[i])
        final = len(data[i])-1
        mag_scale.append((data[i][final] - data[i][0])/2)

    avg_delta = (mag_scale[0] + mag_scale[1] + mag_scale[2]) / 3
    mag_scale[0] = avg_delta / mag_scale[0]
    mag_scale[1] = avg_delta / mag_scale[1]
    mag_scale[2] = avg_delta / mag_scale[2]

    print("Scales: ")
    print(mag_scale[0], mag_scale[1], mag_scale[2])

    return mag_scale

def show_plot():
    #ax.scatter(temp[0], temp[1], temp[2], c="g", marker=".")
    ax.scatter(original[0], original[1], original[2], c="r", marker=".")
    #ax.scatter(dest[0], dest[1], dest[2], c="b", marker=".")
    
    for i in range(3):
        ax.plot(line_x[i], line_y[i], line_z[i])
        
    ax.set_xlabel("X-axis")
    ax.set_xlim(-500, 500)
    ax.set_ylabel("Y-axis")
    ax.set_ylim(-500, 500)
    ax.set_zlabel("Z-axis")
    ax.set_zlim(-500, 500)

    plt.axis('scaled')

    plt.show()

def main():
    read_data()
    #find_average()
    #compare_distances()
    show_plot()
    
main()
