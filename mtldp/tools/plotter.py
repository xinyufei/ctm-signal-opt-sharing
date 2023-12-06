import matplotlib.pyplot as plt


class ColorCycle(object):
    def __init__(self):
        self.color_cycle = plt.rcParams['axes.prop_cycle'].by_key()['color']
        self.current_index = 0
        self.color = self.get_current_color()

    def get_current_color(self):
        return self.color_cycle[self.current_index]

    def next(self):
        self.current_index += 1
        if self.current_index == len(self.color_cycle):
            self.current_index = 0

    def reset(self):
        self.current_index = 0

