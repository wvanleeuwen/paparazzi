#! /usr/bin/python

import sys
import os

from video_viewer import VideoViewer

if __name__ == '__main__':
    filename = os.path.join(os.path.dirname(os.path.abspath(__file__)), sys.argv[1])

    viewer = VideoViewer(filename)
    viewer.scale = float(sys.argv[2])
    viewer.rotate = int(sys.argv[3])
    viewer.write_frames = False
    viewer.run()

