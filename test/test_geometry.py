import numpy as np

from kinect_v2_ros2.geometry import (
    bgrx_to_rgb,
    camera_info_fields,
    float_to_u16,
    points_xyz,
    scaled,
)
from kinect_v2_ros2.simulator import Simulator


def test_bgrx_to_rgb_and_step():
    bgrx = np.zeros((1080, 1920, 4), np.uint8)
    bgrx[..., 0], bgrx[..., 2] = 10, 200  # B, R
    rgb = bgrx_to_rgb(bgrx[::2, ::2])
    assert rgb.shape == (540, 960, 3) and tuple(rgb[0, 0]) == (200, 0, 10)


def test_scaled_intrinsics():
    s = scaled({"fx": 1081.37, "fy": 1081.37, "cx": 959.5, "cy": 539.5}, 0.5)
    info = camera_info_fields(960, 540, **s)
    assert abs(info["k"][0] - 540.685) < 1e-3 and info["k"][2] == 479.75


def test_float_to_u16():
    a = np.array([[np.nan, -1.0, 1234.4, 1e9]], np.float32)
    assert float_to_u16(a).tolist() == [[0, 0, 1234, 65535]]


def test_points():
    d = np.zeros((424, 512), np.uint16)
    d[211, 255] = 1500
    pts = points_xyz(d, step=1, fx=365.5, fy=365.5, cx=255.5, cy=211.5)
    assert pts.shape == (1, 3) and abs(pts[0, 2] - 1.5) < 1e-6


def test_simulator_formats():
    sim = Simulator()
    assert sim.depth(0).shape == (424, 512) and sim.depth(0).dtype == np.float32
    assert sim.color(0, 2).shape == (540, 960, 4)
    assert sim.registered(8.0).shape == (424, 512, 4)
    assert sim.audio(0).shape == (1600, 4)
    assert sim.person(8.0)[2] and not sim.person(1.0)[2]
