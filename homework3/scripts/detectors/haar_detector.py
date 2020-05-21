import cv2
from .face import Face

class HaarDetector():

    def find_faces(self, image):
        min_size = (5, 5)
        max_size = (60, 60)
        haar_scale = 1.2
        min_neighbors = 3
        haar_flags = 0

        # Detect the objects
        results = self.detectorCascade.detectMultiScale(image, haar_scale, min_neighbors, haar_flags, min_size, max_size)

        detections = []

        for (x, y, w, h) in results:
            pt1 = (int(x), int(y))
            pt2 = (int((x + w)), int((y + h)))
            detections.append(Face(pt1, pt2))

        return detections
          
    def __init__(self, data_path):
        # We must provide the path to the haarcascade_face.xml file
        self.detectorCascade = cv2.CascadeClassifier(data_path)