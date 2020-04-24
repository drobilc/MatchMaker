import cv2
import glob, os.path
import numpy
import random, math
import sys

# The image will be resized to this size before doing hough circle detection
IMAGE_SIZE = (640, 480)

# The results will be saved to output_file as r;g;b;class
output_file = open('test.csv', 'w', encoding='utf-8')

def sample_colors(image, n=10, region_size=10):
    # Now that we have selected a circle, sample random points from it
    average_colors = []

    height, width, channels = image.shape

    for i in range(n):
        x = random.random() * (width - 2 * region_size) + region_size
        y = random.random() * (width - 2 * region_size) + region_size

        # For each point, calculate an average color around the center
        region = image[int(y)-region_size:int(y)+region_size,int(x)-region_size:int(x)+region_size]
        average_color = numpy.average(numpy.average(region, axis=0), axis=0)
        average_colors.append(average_color)
    
    return average_colors

# Find directories inside the images/train folder. Each folder should contain images
# and its name is the class name that this script will produce
image_directories = glob.glob('images/test/*')
print('Found {} folders with images'.format(len(image_directories)))

for directory in image_directories:
    directory_name = os.path.basename(directory)

    # Find all images inside current directory
    images = glob.glob(os.path.join(directory, '*.jpg'))

    print('Folder {} contains {} images'.format(directory_name, len(images)))

    for image_path in images:
        # Read the image and resize it to IMAGE_SIZE
        image = cv2.imread(image_path)

        # Sample n points from circle and get their colors
        colors = sample_colors(image, n=20)

        for color in colors:
            color = numpy.around(color, decimals=2)
            # The image is in BGR format, convert it to rgb
            output_file.write('{};{};{};{}\n'.format(color[2], color[1], color[0], directory_name))

output_file.close()