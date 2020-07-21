import dlib
from face import Face

class HogDetector():

    def __init__(self):
        self.detector = dlib.get_frontal_face_detector()

    def find_faces(self, image):
        # We can use dlib face detection to also return detected face probablities
        # the first argument is the image, the second is the upscale parameter, that
        # tells dlib how much to upscale image to detect faces and third parameter
        # is the minimum probability of detected face
        faces, scores, idx = self.detector.run(image, 0, 0.25)
        return faces

class CnnDetector():

    def __init__(self, data_path):
        self.detector = dlib.cnn_face_detection_model_v1(data_path)
    
    def find_faces(self, image):
        detections = self.detector(image, 0)
        rects = dlib.rectangles()
        rects.extend([d.rect for d in detections])
        return rects