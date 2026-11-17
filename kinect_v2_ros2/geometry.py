"""Camera models, ROS message helpers and point cloud – numpy only, unit-tested."""
import numpy as np

COLOR_W, COLOR_H = 1920, 1080
DEPTH_W, DEPTH_H = 512, 424
# Nominal intrinsics – the driver reads the real ones from the device (factory calibration).
DEPTH_PARAMS = {"fx": 365.5, "fy": 365.5, "cx": 255.5, "cy": 211.5}
COLOR_PARAMS = {"fx": 1081.37, "fy": 1081.37, "cx": 959.5, "cy": 539.5}


def scaled(params, factor):
    """Intrinsics of an image scaled by factor (e.g. 0.5 for 960 x 540)."""
    return {k: v * factor for k, v in params.items()}


def camera_info_fields(width, height, fx, fy, cx, cy):
    """Values for sensor_msgs/CameraInfo (plumb bob, distortion already removed)."""
    return {
        "width": width, "height": height, "distortion_model": "plumb_bob",
        "d": [0.0] * 5,
        "k": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
    }


def image_fields(array, encoding):
    """Values for sensor_msgs/Image from a numpy array (rgb8, mono16, 16UC1)."""
    array = np.ascontiguousarray(array)
    height, width = array.shape[:2]
    return {"height": height, "width": width, "encoding": encoding, "is_bigendian": 0,
            "step": array.strides[0], "data": array.tobytes()}


def bgrx_to_rgb(bgrx, step=1):
    """libfreenect2 delivers BGRX (4 bytes per pixel) – thin out and reorder to RGB."""
    return np.ascontiguousarray(bgrx[::step, ::step, 2::-1])


def float_to_u16(img, scale=1.0):
    """Float frames (depth in mm, IR 0 ... 65535) -> uint16, NaN/inf -> 0."""
    out = np.nan_to_num(img * scale, nan=0.0, posinf=0.0, neginf=0.0)
    return np.clip(out, 0, 65535).astype(np.uint16)


def points_xyz(depth_mm, step=2, fx=365.5, fy=365.5, cx=255.5, cy=211.5, rgb=None):
    """Depth image in mm -> N x 3 float32 points in metres (optical frame, z forward)."""
    d = depth_mm[::step, ::step].astype(np.float32) / 1000.0
    v, u = np.mgrid[0:depth_mm.shape[0]:step, 0:depth_mm.shape[1]:step]
    valid = d > 0
    z = d[valid]
    pts = np.column_stack([(u[valid] - cx) * z / fx, (v[valid] - cy) * z / fy, z])
    pts = pts.astype(np.float32)
    if rgb is None:
        return pts
    c = rgb[::step, ::step][valid].astype(np.uint32)
    packed = ((c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]).view(np.float32)
    return np.column_stack([pts, packed])
