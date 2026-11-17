"""Synthetic Kinect v2 scene in the native formats of libfreenect2 (BGRX colour, float depth/IR)."""
import math

import numpy as np

from .geometry import COLOR_H, COLOR_W, DEPTH_H, DEPTH_W


class Simulator:
    def __init__(self, seed=11):
        self.rng = np.random.default_rng(seed)
        self.dy, self.dx = np.mgrid[0:DEPTH_H, 0:DEPTH_W]

    def person(self, t):
        cx = 0.5 + 0.3 * math.sin(t * 0.5)          # horizontal position 0 ... 1
        dist = 1500 + 1000 * (1 + math.sin(t * 0.7)) / 2  # 1.5 ... 2.5 m
        hand = (t % 10.0) > 7.0                     # hand raised 3 s of every 10 s
        return cx, dist, hand

    def _boxes(self, t, w, h):
        cx, dist, hand = self.person(t)
        c = int(cx * w)
        boxes = [(int(.33 * h), int(.95 * h), c - int(.09 * w), c + int(.09 * w), dist),
                 (int(.21 * h), int(.33 * h), c - int(.04 * w), c + int(.04 * w), dist)]
        if hand:
            boxes.append((int(.08 * h), int(.27 * h), c + int(.1 * w), c + int(.16 * w),
                          dist - 400))
        return boxes

    def depth(self, t):
        d = np.full((DEPTH_H, DEPTH_W), 3500.0, np.float32)    # wall at 3.5 m
        d[:, :20] = 0                                         # invalid border, as on the device
        for y0, y1, x0, x1, z in self._boxes(t, DEPTH_W, DEPTH_H):
            d[y0:y1, max(x0, 0):x1] = z
        return d

    def ir(self, t):
        base = 60000.0 - self.depth(t) * 12
        noise = self.rng.normal(0, 1500, base.shape)
        return np.clip(base + noise, 0, 65535).astype(np.float32)

    def color(self, t, step=1):
        """BGRX colour image, 1920x1080 thinned out by step."""
        return self._render(t, COLOR_W // step, COLOR_H // step)

    def registered(self, t):
        """Colour mapped onto the 512x424 depth image (BGRX), invalid border black."""
        img = self._render(t, DEPTH_W, DEPTH_H)
        img[:, :20] = 0
        return img

    def _render(self, t, w, h):
        yy, xx = np.mgrid[0:h, 0:w]
        img = np.zeros((h, w, 4), np.uint8)
        img[..., 0] = 128                                     # B
        img[..., 1] = (yy * 255 // max(h - 1, 1))             # G
        img[..., 2] = (xx + int(t * 80)) % 256                # R
        for i, (y0, y1, x0, x1, _) in enumerate(self._boxes(t, w, h)):
            img[y0:y1, max(x0, 0):x1, :3] = (200, 90, 40) if i == 0 else (160, 190, 230)
        return img

    def audio(self, t, frames=1600, rate=16000, channels=4):
        k = np.arange(frames) / rate + t
        mono = (3000 * np.sin(2 * math.pi * 440 * k)).astype(np.int16)
        return np.repeat(mono[:, None], channels, axis=1)
