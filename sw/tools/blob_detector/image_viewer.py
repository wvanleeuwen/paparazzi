import cv2

from viewer import Viewer


class ImageViewer(Viewer):
    image = None

    def __init__(self, image_file):
        self.image = cv2.imread(image_file, cv2.IMREAD_COLOR)

    def read_frame(self):
        return True, self.image.copy()
