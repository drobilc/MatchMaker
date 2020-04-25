from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy, csv
from colormath.color_objects import sRGBColor, HSVColor, LabColor
from colormath.color_conversions import convert_color

def read_data(input_file):
    # This function reads data from input file of format r;g;b;class
    rows = csv.reader(csv_file, delimiter = ';')

    data_x, data_y = [], []
    for row in rows:
        data_x.append([float(row[0]), float(row[1]), float(row[2])])
        data_y.append(row[3])

    data_x = numpy.asarray(data_x)
    data_y = numpy.asarray(data_y)
    return data_x, data_y

with open('train.csv', 'r', encoding='utf-8') as csv_file:
    colors, labels = read_data(csv_file)

# After RGB colors have been read, convert colors to hsv and lab
def rgb_to_hsv(color):
    hsv = convert_color(sRGBColor(*(color / 255)), HSVColor)
    return numpy.asarray([hsv.hsv_h, hsv.hsv_s, hsv.hsv_v])

def rgb_to_lab(color):
    lab = convert_color(sRGBColor(*(color / 255)), LabColor)
    return numpy.asarray([lab.lab_l, lab.lab_a, lab.lab_b])

hsv_colors = numpy.apply_along_axis(rgb_to_hsv, 1, colors)
lab_colors = numpy.apply_along_axis(rgb_to_lab, 1, colors)

# Extract red, green and blue data
reds, greens, blues = colors[:,0], colors[:,1], colors[:,2]
hues, saturations, values = hsv_colors[:,0], hsv_colors[:,1], hsv_colors[:,2]
l, a, b = lab_colors[:,0], lab_colors[:,1], lab_colors[:,2]

figure = plt.figure()
ax = figure.add_subplot(projection='3d')
ax.scatter3D(reds, greens, blues, color=colors / 255)
ax.set_xlabel('red')
ax.set_ylabel('green')
ax.set_zlabel('blue')
plt.savefig('rgb.png', dpi=320)

figure = plt.figure()
ax = figure.add_subplot(projection='3d')
ax.scatter3D(hues, saturations, values, color=colors / 255)
ax.set_xlabel('hue')
ax.set_ylabel('saturation')
ax.set_zlabel('value')
plt.savefig('hsv.png', dpi=320)

figure = plt.figure()
ax = figure.add_subplot(projection='3d')
ax.scatter3D(l, a, b, color=colors / 255)
ax.set_xlabel('lightness')
ax.set_ylabel('a')
ax.set_zlabel('b')
plt.savefig('lab.png', dpi=320)