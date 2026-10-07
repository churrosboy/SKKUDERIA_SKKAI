#!/usr/bin/env python3
"""Paint over bad blobs in a saved occupancy-grid map PNG and save it in place.

Usage:
    python3 map_painter.py GLC_smile_small            # map name in stack_master/maps
    python3 map_painter.py /path/to/some_map.png      # or a direct path

Controls:
    left-drag        paint with the selected value
    radio buttons    Occupied (0) / Free (255) / Unknown (205)
    slider           brush radius in pixels
    u / Undo button  undo last stroke
    s / Save button  overwrite the PNG and pf_map.png, then exit
                     (first save keeps a one-time .bak)
    toolbar zoom/pan work as usual; painting is disabled while they are active
"""
import argparse
import os
import shutil
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, RadioButtons, Slider

PAINT_VALUES = {'Occupied': 0, 'Free': 255, 'Unknown': 205}
MAPS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'maps')


def resolve_map_path(arg: str) -> str:
    if os.path.isfile(arg):
        return arg
    candidate = os.path.join(MAPS_DIR, arg, arg + '.png')
    if os.path.isfile(candidate):
        return candidate
    sys.exit(f"Map not found: neither '{arg}' nor '{candidate}' exists")


class MapPainter:
    def __init__(self, img_path: str):
        self.img_path = img_path
        # 's' must trigger our save-and-exit, not matplotlib's save-figure dialog
        plt.rcParams['keymap.save'] = []
        self.img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if self.img is None:
            sys.exit(f'Failed to read image: {img_path}')
        self.undo_stack = []
        self.paint_value = PAINT_VALUES['Occupied']
        self.brush_radius = 3
        self.last_pt = None
        self.stroke_active = False

        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        self.fig.canvas.manager.set_window_title(f'Map painter - {os.path.basename(img_path)}')
        self.fig.subplots_adjust(left=0.22, bottom=0.15)
        self.im_artist = self.ax.imshow(self.img, cmap='gray', vmin=0, vmax=255,
                                        interpolation='nearest')
        self.ax.set_title(img_path, fontsize=8)

        ax_radio = self.fig.add_axes([0.02, 0.6, 0.16, 0.15])
        self.radio = RadioButtons(ax_radio, list(PAINT_VALUES))
        self.radio.on_clicked(self.on_color)

        ax_slider = self.fig.add_axes([0.3, 0.05, 0.4, 0.03])
        self.slider = Slider(ax_slider, 'Brush [px]', 1, 30, valinit=self.brush_radius, valstep=1)
        self.slider.on_changed(self.on_brush)

        ax_undo = self.fig.add_axes([0.02, 0.45, 0.16, 0.06])
        self.btn_undo = Button(ax_undo, 'Undo (u)')
        self.btn_undo.on_clicked(lambda _: self.undo())

        ax_save = self.fig.add_axes([0.02, 0.35, 0.16, 0.06])
        self.btn_save = Button(ax_save, 'Save (s)')
        self.btn_save.on_clicked(lambda _: self.save())

        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

    # ---- widget callbacks -------------------------------------------------
    def on_color(self, label):
        self.paint_value = PAINT_VALUES[label]

    def on_brush(self, val):
        self.brush_radius = int(val)

    def on_key(self, event):
        if event.key == 'u':
            self.undo()
        elif event.key == 's':
            self.save()

    # ---- painting ---------------------------------------------------------
    def _toolbar_busy(self) -> bool:
        toolbar = getattr(self.fig.canvas, 'toolbar', None)
        return bool(toolbar is not None and toolbar.mode)

    def on_press(self, event):
        if event.inaxes is not self.ax or event.button != 1 or self._toolbar_busy():
            return
        self.undo_stack.append(self.img.copy())
        del self.undo_stack[:-30]
        self.stroke_active = True
        self.last_pt = (int(event.xdata), int(event.ydata))
        self._dab(self.last_pt)

    def on_motion(self, event):
        if not self.stroke_active or event.inaxes is not self.ax:
            return
        pt = (int(event.xdata), int(event.ydata))
        cv2.line(self.img, self.last_pt, pt, self.paint_value,
                 thickness=max(1, 2 * self.brush_radius))
        self.last_pt = pt
        self.refresh()

    def on_release(self, _event):
        self.stroke_active = False
        self.last_pt = None

    def _dab(self, pt):
        cv2.circle(self.img, pt, self.brush_radius, self.paint_value, thickness=-1)
        self.refresh()

    def refresh(self):
        self.im_artist.set_data(self.img)
        self.fig.canvas.draw_idle()

    def undo(self):
        if not self.undo_stack:
            print('Nothing to undo')
            return
        self.img = self.undo_stack.pop()
        self.refresh()

    # ---- saving -----------------------------------------------------------
    def save(self):
        bak = self.img_path + '.bak'
        if not os.path.exists(bak):
            shutil.copy2(self.img_path, bak)
            print(f'Backup of the original written to {bak}')
        cv2.imwrite(self.img_path, self.img)
        print(f'Saved {self.img_path}')

        pf_path = os.path.join(os.path.dirname(self.img_path), 'pf_map.png')
        if os.path.abspath(pf_path) != os.path.abspath(self.img_path):
            cv2.imwrite(pf_path, self.img)
            print(f'Saved {pf_path}')

        plt.close(self.fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('map', help='map name (folder in stack_master/maps) or path to a .png')
    args = parser.parse_args()

    painter = MapPainter(resolve_map_path(args.map))
    print("Paint with left-drag. 'u' = undo, 's' = save map + pf_map and exit.")
    plt.show()


if __name__ == '__main__':
    main()
