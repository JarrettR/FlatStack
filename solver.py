from layers import Layers, Layer
import kiwisolver


class Solver(object):
    def __init__(self, layer):
        self.layer = layer
