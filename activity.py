#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# SprayPlay
# Copyright (C) 2008, Brian Jordan, Gregory Jordan, Eric Jordan
# Copyright (C) 2012, Alan Aguiar
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# Contact information:
# Brian Jordan <bcjordan@gmail.com>
# Gregory Jordan <gjuggler@gmail.com>
# Eric Jordan <ewjordan@gmail.com>
# Alan Aguiar <alanjas@gmail.com>


import gtk
import sugargame.canvas
from sugar.activity import activity
from gettext import gettext as _

from sugar.graphics.toolbarbox import ToolbarBox
from sugar.activity.widgets import ActivityToolbarButton
from sugar.activity.widgets import StopButton
from sugar.graphics.toolbutton import ToolButton

import sprayplay

class SprayPlay(activity.Activity):

    def __init__(self, handle):
        activity.Activity.__init__(self, handle)
        
        self.game = sprayplay.SprayPlay()
        self.build_toolbar()
        self._pygamecanvas = sugargame.canvas.PygameCanvas(self)
        self.set_canvas(self._pygamecanvas)
        self._pygamecanvas.grab_focus()
        self._pygamecanvas.run_pygame(self.game.run)

    def build_toolbar(self):

        toolbox = ToolbarBox()
        activity_button = ActivityToolbarButton(self)
        toolbox.toolbar.insert(activity_button, -1)
        activity_button.show()

        barra = toolbox.toolbar

        """
        separator1 = gtk.SeparatorToolItem()
        separator1.props.draw = True
        separator1.set_expand(False)
        barra.insert(separator1,-1)

        save_button = ToolButton('filesave')
        save_button.set_tooltip(_('Save Image'))
        save_button.connect('clicked', self._savebutton_cb)
        barra.insert(save_button, -1)
        save_button.show()"""

        separator2 = gtk.SeparatorToolItem()
        separator2.props.draw = False
        separator2.set_expand(True)
        barra.insert(separator2, -1)

        stop_button = StopButton(self)
        stop_button.props.accelerator = '<Ctrl>q'
        barra.insert(stop_button, -1)
        stop_button.show()

        self.set_toolbar_box(toolbox)

        toolbox.show_all()


