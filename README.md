# ROS2-Kinect-v2-Xbox-One

ROS 2 driver node for the **Kinect for Xbox One** (Kinect v2, also the "Kinect for Windows v2")
based on [libfreenect2](https://github.com/OpenKinect/libfreenect2) and
[pylibfreenect2](https://github.com/r9y9/pylibfreenect2). One node publishes every stream once –
any number of applications subscribe to what they need.

Unlike the Kinect v1, the v2 delivers **colour, infrared and depth at the same time**.

`simulate:=true` produces a synthetic scene with exactly the same topics (a person walking
around who raises a hand for 3 s every 10 s) – everything can be developed without a Kinect.

Tested with ROS 2 **Jazzy** and **Lyrical** (Ubuntu 24.04/26.04, Raspberry Pi 5).

```
                            /kinect_v2/color/image_raw       rgb8 960x540 (color_step 2)  ~47 MB/s
 Kinect v2 ─ libfreenect2 ─ driver ──► /kinect_v2/depth/image_raw  16UC1 512x424 mm      ~13 MB/s
                                   ──► /kinect_v2/ir/image_raw     mono16 512x424         ~13 MB/s
                                   ──► /kinect_v2/registered/image_raw  rgb8 512x424      ~20 MB/s
                                   ──► /kinect_v2/depth/points     PointCloud2 xyz+rgb 10 Hz
                                   ──► /kinect_v2/{color,depth,ir}/camera_info, /kinect_v2/audio
```

All streams at 30 Hz add up to about **100 MB/s** – fine on one computer (shared memory), too
much for Wi-Fi. Over the network use `color_step: 4`, turn off what you don't need, or
compress with `image_transport` (see the book).

## Hardware notes

* **USB 3.0** controller (Intel or Renesas recommended) and the Kinect adapter with power
  supply. The Xbox One version needs the "Kinect Adapter for Windows".
* Depth decoding is expensive: `pipeline: opengl` or `opencl` uses the GPU, `cpu` works
  everywhere. A Raspberry Pi 5 manages colour, depth and IR with `color_step: 2` or more.
* libfreenect2 has **no skeleton tracking** (that was part of Microsoft's SDK). Use MediaPipe
  or similar on `color/image_raw` – see *Smart Home & Kinect* and *Smart Home & MediaPipe*.
* Intrinsics are read from the device (factory calibration) and published as `camera_info`.

## Installation

```bash
# libfreenect2
sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev libturbojpeg0-dev \
                 libglfw3-dev libopenni2-dev git python3-numpy python3-dev cython3
git clone https://github.com/OpenKinect/libfreenect2.git && cd libfreenect2
mkdir build && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make -j"$(nproc)" && make install
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/   # access without sudo
./bin/Protonect                                                     # hardware test

# Python wrapper
export PKG_CONFIG_PATH=$HOME/freenect2/lib/pkgconfig LIBFREENECT2_INSTALL_PREFIX=$HOME/freenect2
export LD_LIBRARY_PATH=$HOME/freenect2/lib:$LD_LIBRARY_PATH          # also in ~/.bashrc
pip install --break-system-packages git+https://github.com/r9y9/pylibfreenect2.git

# this package
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Michdo93/ROS2-Kinect-v2-Xbox-One.git
cd .. && source /opt/ros/lyrical/setup.bash                         # or jazzy
colcon build --symlink-install && source install/setup.bash
```

Microphone array (optional): the Kinect v2 is a standard USB audio device with 4 channels –
`pip install --break-system-packages sounddevice` and `audio: true`.

## Usage

```bash
ros2 launch kinect_v2_ros2 kinect_v2.launch.py                 # hardware
ros2 launch kinect_v2_ros2 kinect_v2.launch.py simulate:=true  # no Kinect needed
ros2 run rqt_image_view rqt_image_view /kinect_v2/color/image_raw
rviz2   # PointCloud2 /kinect_v2/depth/points, fixed frame kinect_v2_link
```

Parameters (`config/kinect_v2.yaml`):

| Parameter | Default | Meaning |
|---|---|---|
| `simulate` | false | synthetic scene instead of the hardware |
| `serial` | "" | serial number when several Kinects are connected |
| `pipeline` | cpu | depth decoding: cpu, opengl, opencl, cuda |
| `frame_prefix` | kinect_v2 | frames `<prefix>_link`, `_rgb_optical_frame`, `_ir_optical_frame` |
| `color_step` | 2 | 1: 1920×1080, 2: 960×540, 4: 480×270 |
| `publish_ir`, `publish_registered` | true | optional streams |
| `cloud_step` | 2 | point cloud from every 2nd pixel (256 × 212 points) |
| `cloud_every` | 3 | point cloud every 3rd frame (only while someone subscribes) |
| `reliable` | true | `false`: best-effort QoS (sensor data) – better over Wi-Fi |
| `audio`, `audio_device`, `audio_channels` | false, "", 4 | microphone array via ALSA |
| `fps` | 30.0 | simulation only |

## Tests

```bash
python3 -m pytest -q test          # geometry, point cloud, simulator – no ROS, no Kinect
```

## Related

* [ROS2-Kinect-v1-Xbox-360](https://github.com/Michdo93/ROS2-Kinect-v1-Xbox-360) – the same for
  the Kinect v1
* [ros2_openhab_ws](https://github.com/Michdo93/ros2_openhab_ws) – bridge between ROS 2 and
  openHAB
* Book *Smart Home & ROS2* (Praxisreihe Smart Home) by Michael Christian Dörflinger

## License

MIT
