#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rqt_gui_py.plugin import Plugin

from src.robot_controller.mico_controller.src.rqt_mico_buttons.mico_buttons_widget import MicoButtonsWidget


class MicoButtons(Plugin):
    def __init__(self, context):
        super(MicoButtons, self).__init__(context)
        self.setObjectName('MicoButtons')
        self._widget = MicoButtonsWidget(self)
        self._widget.start()
        context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()