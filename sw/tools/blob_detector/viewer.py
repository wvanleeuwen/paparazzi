import cv2
import numpy as np
from matplotlib import pyplot as plt
from settings import Settings


settings = Settings([
    {'name': 'Ymin', 'value': 0, 'max': 255},
    {'name': 'Ymax', 'value': 255, 'max': 255},
    {'name': 'Umin', 'value': 0, 'max': 255},
    {'name': 'Umax', 'value': 255, 'max': 255},
    {'name': 'Vmin', 'value': 0, 'max': 255},
    {'name': 'Vmax', 'value': 255, 'max': 255}
])


class Viewer(object):
    frame = None
    scale = 1
    yuv_frame = None
    mouse = dict()

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print 'clicked at: [%i %i], YUV: %s' % (x, y, self.yuv_frame[y][x])
            self.mouse['start'] = None

        if event == cv2.EVENT_RBUTTONDOWN:
            self.mouse['start'] = (x, y)

        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse['now'] = (x, y)

        if event == cv2.EVENT_RBUTTONUP:
            if not self.mouse.get('start'):
                return

            sx, sy = self.mouse['start']
            self.mouse['start'] = None

            mask = np.zeros(self.frame.shape[:2], np.uint8)
            mask[sy:y, sx:x] = 255

            channels = ('Y', 'U', 'V')
            for i, label in enumerate(channels):
                mean = self.yuv_frame[sy:y, sx:x, i].mean()
                std = self.yuv_frame[sy:y, sx:x, i].std()

                settings[label + 'min'] = int(mean - 4*std)
                settings[label + 'max'] = int(mean + 4*std)

    def run(self):

        while True:
            ret, self.frame = self.read_frame()

            if not ret:
                break
            key = cv2.waitKey(10) & 0xFF

            if key == ord('q'):
                break

            if key == ord('c'):
                print "--- filter settings ---"
                for c in ('Y', 'U', 'V'):
                    print "filter.%s_min = %i;" % (c.lower(), settings[c + 'min'])
                    print "filter.%s_max = %i;" % (c.lower(), settings[c + 'max'])

            self.yuv_frame = cv2.cvtColor(self.frame.copy(), cv2.COLOR_RGB2YUV)

            a = (settings['Ymin'], settings['Umin'], settings['Vmin'])
            b = (settings['Ymax'], settings['Umax'], settings['Vmax'])
            filtered_frame = cv2.inRange(self.yuv_frame, a, b)

            if self.mouse.get('start'):
                cv2.rectangle(self.frame, self.mouse['start'], self.mouse['now'], (0, 255, 0), 2)

            view = np.concatenate((self.frame, cv2.cvtColor(filtered_frame, cv2.COLOR_GRAY2BGR)), axis=1)

            if self.scale != 1:
                h, w = view.shape[:2]
                view = cv2.resize(view, (int(self.scale * w), int(self.scale * h)))

            cv2.imshow('settings', view)

    def read_frame(self):
        raise NotImplementedError
