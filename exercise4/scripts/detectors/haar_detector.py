import cv2
from face import Face

class HaarDetector():

    def find_faces(self, image):
        # min_size = (30,30)
        min_size = (5, 5)
        max_size = (60, 60)
        haar_scale = 1.25
        min_neighbors = 3
        haar_flags = 0

        # Convert color input image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Scale input image for faster processing
        if self.image_scale == None:
            self.image_scale = image.shape[1] / 240

        smallImage = cv2.resize(gray, (int(image.shape[1] / self.image_scale), int(image.shape[0] / self.image_scale)), interpolation=cv2.INTER_LINEAR)

        # Equalize the histogram (DOESN'T WORK OK)
        # smallImage = cv2.equalizeHist(smallImage)

        # Detect the objects
        results = self.detectorCascade.detectMultiScale(smallImage, haar_scale, min_neighbors, haar_flags, min_size, max_size)

        detections = []

        for (x, y, w, h) in results:
            pt1 = (int(x * self.image_scale), int(y * self.image_scale))
            pt2 = (int((x + w) * self.image_scale), int((y + h) * self.image_scale))
            detections.append(Face(pt1, pt2))

        return detections
          
    def __init__(self, data_path):
        # Image scale tells us how much the image should be resized when finding faces
        self.image_scale = 1.5
        # We must provide the path to the haarcascade_face.xml file
        self.detectorCascade = cv2.CascadeClassifier(data_path)