#!/usr/bin/env python
# -*- coding: utf-8 -*-
class Axis(object):
    def __init__(self, x=0.0, y=0.0):
        self.set_data(x, y)

    def set_data(self, x, y):
        self.x, self.y = x, y

    def __str__(self):
        return "[{x:5.2f}, {y:5.2f}]".format(x=self.x, y=self.y)


class Button(object):
    def __init__(self, buttons=[0] * 12):
        self.set_data(buttons)
        self._label_list = ("□", "✕", "○", "△",  # 0-3
                            "L1", "R1", "L2", "R2",  # 4-7
                            "start", "select",  # 8-9
                            "L3", "R3")  # 10-11

    def set_data(self, buttons):
        self.buttons = buttons

    def is_pressed(self, button_label, is_only=False, is_any=False):
        _labels = button_label if isinstance(button_label, list) else [button_label]
        if is_only:
            _other_labels = list(set(self._label_list) - set(_labels))
            return (all([self.buttons[self._label_list.index(label)] == 1 for label in _labels]) is True) \
                   and (any([self.buttons[self._label_list.index(label)] == 1 for label in _other_labels]) is False)
        elif is_any:
            return any([self.buttons[self._label_list.index(label)] == 1 for label in _labels])
        else:
            return all([self.buttons[self._label_list.index(label)] == 1 for label in _labels])

    def __str__(self):
        return str([label for idx, label in self._label_list if self.buttons[idx] == 1])


class MyJoyData(object):
    def __init__(self, axis_list=[0.0] * 6, button_list=[0] * 12):
        self.set_data(axis_list, button_list)

    def set_data(self, axis_list, button_list):
        self.lstick = Axis(axis_list[1], axis_list[0])
        self.rstick = Axis(axis_list[3], axis_list[2])
        self.dpad = Axis(axis_list[5], axis_list[4])
        self.buttons = Button(button_list)

    def is_pressed(self, buton_label, is_only=False, is_any=False):
        return self.buttons.is_pressed(buton_label, is_only, is_any)
