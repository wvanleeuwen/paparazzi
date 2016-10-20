import cv2

def _nothing_(x):
    pass


class Settings(object):
    def __init__(self, settings_array):

        cv2.namedWindow('settings')

        self.settings = settings_array

        for st in self.settings:
            cv2.createTrackbar(st['name'], 'settings', st['value'], st['max'], _nothing_)

    def __getitem__(self, name):
        return cv2.getTrackbarPos(name, 'settings')

    def __setitem__(self, name, value):
        cv2.setTrackbarPos(name, 'settings', value)

    def print_settings(self):
        for st in self.settings:
            print "{'name': '%s', 'value': %i, 'max': %i}," % \
                  (st['name'], self[st['name']], st['max'])


