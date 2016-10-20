import cv2
import datetime
import os

from viewer import Viewer


class VideoViewer(Viewer):
    cap = None
    index = 0
    rotate = True
    write_frames = True

    def __init__(self, video_file):
        self.cap = cv2.VideoCapture(video_file)
        self.folder = datetime.datetime.now().isoformat()

        if self.write_frames:
            os.mkdir(self.folder)

        if not self.cap.isOpened():
            raise IOError("Cannot open video stream")

    def read_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            return ret, frame

        if self.rotate:
            frame = cv2.transpose(frame)
            frame = cv2.flip(frame, 0)

        if self.write_frames:
            name = self.folder + ("/%05d" % self.index) + ".jpg"
            cv2.imwrite(name, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            self.index += 1

        return ret, frame
