import cv2
import glob, os.path
import numpy
import random, math
import sys

# The image will be resized to this size before doing hough circle detection
IMAGE_SIZE = (640, 480)

def find_color_circle(image):
    # The HoughCircles method must be performed on grayscale image, so we convert it here
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Find circles in the image
    circles = cv2.HoughCircles(
        grayscale,                                  # The grayscale image
        method = cv2.HOUGH_GRADIENT,                # Detection method (only one available)
        dp = 1,                                     # Inverse ratio of the accumulator resolution to the image resolution (1 / dp is resolution)
        minDist = 40,                               # Minimum distance between the centers of the detected circles
        param1 = 100,                               # The higher threshold of the two passed to the Canny edge detector
        param2 = 30,                                # Accumulator threshold for the circle centers at the detection stage
        minRadius = 20,                             # Minimum circle radius
        maxRadius = int(IMAGE_SIZE[1] / 2 * 0.8)    # Maximum circle radius
    )

    # Round circle coordinates to and convert to integers
    circles = numpy.uint16(numpy.around(circles))
    circles = circles[0,:]

    # Find the closest circle to the center of the image
    if len(circles) <= 0:
        return None

    order = numpy.square(circles[:,0] - IMAGE_SIZE[0] / 2) + numpy.square(circles[:,1] - IMAGE_SIZE[1] / 2)
    indices = numpy.argsort(order)
    circles = circles[indices]

    # Get only the closest circle and return it
    return circles[0]

def sample_white_colors(image, circle, n=10, region_size=10):
    average_colors = []

    height, width, channels = image.shape

    # Pick n colors randomly OUTSIDE the circle
    while len(average_colors) < n:
        # Randomly pick a point inside the full image
        x = random.random() * (width - 2 * region_size) + region_size
        y = random.random() * (height - 2 * region_size) + region_size

        circle_radius = circle[2] * 1.1
        if ((circle[0] - x) ** 2 + (circle[1] - y) ** 2 <= circle_radius ** 2):
            continue

        region = image[int(y)-region_size:int(y)+region_size,int(x)-region_size:int(x)+region_size]
        average_color = numpy.average(numpy.average(region, axis=0), axis=0)
        average_colors.append(average_color)

    return average_colors

def sample_colors(image, circle, n=10, region_size=10):
    # Now that we have selected a circle, sample random points from it
    average_colors = []

    for i in range(n):
        # Generate a random point inside the circle (but try to avoid choosing
        # colors from the edge of the circle because it may not be the best)
        # Instead of using the full radius of the circle, use 90% of it
        circle_radius = circle[2] * 0.9
        # Don't choose points uniformly from the circle but prefer points closer
        # to the center. If we wanted to choose uniformly, we would need to
        # change random_radius = circle[2] * math.sqrt(random.random())
        random_angle = random.random() * 2 * math.pi
        random_radius = circle_radius * random.random()
        x = math.cos(random_angle) * random_radius + circle[0]
        y = math.sin(random_angle) * random_radius + circle[1]

        # For each point, calculate an average color around the center
        region = image[int(y)-region_size:int(y)+region_size,int(x)-region_size:int(x)+region_size]
        average_color = numpy.average(numpy.average(region, axis=0), axis=0)
        average_colors.append(average_color)
    
    return average_colors

# Find directories inside the images/train folder. Each folder should contain images
# and its name is the class name that this script will produce
image_directories = glob.glob('images/train/*')
print('Found {} folders with images'.format(len(image_directories)))

data = []

for directory in image_directories:
    directory_name = os.path.basename(directory)

    # Find all images inside current directory
    images = glob.glob(os.path.join(directory, '*.jpg'))

    print('Folder {} contains {} images'.format(directory_name, len(images)))

    for image_path in images:
        # Read the image and resize it to IMAGE_SIZE
        image = cv2.imread(image_path)
        image = cv2.resize(image, IMAGE_SIZE)

        try:
            # Find the circle closest to the center
            circle = find_color_circle(image)
            
            # Sample n points from circle and get their colors
            colors = sample_colors(image, circle, n=50)
            white_colors = sample_white_colors(image, circle, n=5)

            for color in colors:
                color = numpy.around(color, decimals=2)
                # The image is in BGR format, convert it to rgb
                data.append([color[2], color[1], color[0], directory_name])
                # output_file.write('{};{};{};{}\n'.format(color[2], color[1], color[0], directory_name))
            
            for white in white_colors:
                color = numpy.around(white, decimals=2)
                data.append([color[2], color[1], color[0], 'white'])
                # output_file.write('{};{};{};{}\n'.format(color[2], color[1], color[0], 'white'))

            # Display the image
            # cv2.circle(image, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
            # cv2.imshow('Image', image)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

        except Exception as e:
            # cv2.imshow('Image', image)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            print(e)

# The results will be saved to output_file as r;g;b;class
output_file = open('train.csv', 'w', encoding='utf-8')

color_data = {}

for element in data:
    label = element[-1]
    if label not in color_data:
        color_data[label] = []
    color_data[label].append((element[0], element[1], element[2]))

minimal_number_of_colors = min([len(color_data[color_name]) for color_name in color_data])

for color_name in color_data:
    selected_colors = random.sample(color_data[color_name], k=minimal_number_of_colors)
    for color in selected_colors:
        output_file.write('{};{};{};{}\n'.format(color[0], color[1], color[2], color_name))

output_file.close()