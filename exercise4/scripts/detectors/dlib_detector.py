import dlib
from face import Face

class DlibDetector():

    def __init__(self):
        self.detector = dlib.get_frontal_face_detector()

    def find_faces(self, image):
        return self.detector(image, 0)