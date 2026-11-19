"""kinect_v2_node – publishes every stream of a Kinect for Xbox One as ROS 2 topics.

Unlike the Kinect v1, the v2 delivers colour, infrared and depth at the same time.

Topics (below the node namespace, default /kinect_v2)
    color/image_raw        sensor_msgs/Image   rgb8, 1920x1080 scaled by color_step
    color/camera_info
    ir/image_raw           sensor_msgs/Image   mono16 512x424 (works in darkness)
    depth/image_raw        sensor_msgs/Image   16UC1 512x424, millimetres, undistorted
    depth/camera_info, ir/camera_info
    registered/image_raw   sensor_msgs/Image   rgb8 512x424, colour mapped onto depth
    depth/points           sensor_msgs/PointCloud2  xyz (+rgb), thinned out by cloud_step
    audio                  std_msgs/Int16MultiArray 16 kHz PCM, 4-channel mic array (ALSA)

simulate:=true produces a synthetic scene with the same topics – no Kinect needed.
libfreenect2 has no skeleton tracking; use MediaPipe or similar on color/image_raw.
"""

import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Int16MultiArray, MultiArrayDimension
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .geometry import (
    COLOR_PARAMS,
    DEPTH_PARAMS,
    bgrx_to_rgb,
    camera_info_fields,
    float_to_u16,
    image_fields,
    points_xyz,
    scaled,
)
from .simulator import Simulator


class KinectV2Node(Node):
    def __init__(self):
        super().__init__("driver")
        dp = self.declare_parameter
        self.simulate = dp("simulate", False).value
        self.serial = dp("serial", "").value
        self.pipeline = dp("pipeline", "cpu").value  # cpu | opengl | opencl | cuda
        self.frame_prefix = dp("frame_prefix", "kinect_v2").value
        self.fps = float(dp("fps", 30.0).value)
        self.color_step = int(dp("color_step", 2).value)  # 1: 1920x1080, 2: 960x540
        self.publish_ir = dp("publish_ir", True).value
        self.publish_registered = dp("publish_registered", True).value
        self.cloud_step = int(dp("cloud_step", 2).value)
        self.cloud_every = int(dp("cloud_every", 3).value)
        self.audio_enabled = dp("audio", False).value
        self.audio_device = dp("audio_device", "").value
        self.audio_channels = int(dp("audio_channels", 4).value)
        reliable = dp("reliable", True).value

        qos = QoSProfile(depth=2) if reliable else qos_profile_sensor_data
        pub = lambda t, typ: self.create_publisher(typ, t, qos)  # noqa: E731
        self.pub_color, self.pub_color_info = pub("color/image_raw", Image), pub(
            "color/camera_info", CameraInfo
        )
        self.pub_ir, self.pub_ir_info = pub("ir/image_raw", Image), pub(
            "ir/camera_info", CameraInfo
        )
        self.pub_depth, self.pub_depth_info = pub("depth/image_raw", Image), pub(
            "depth/camera_info", CameraInfo
        )
        self.pub_registered = pub("registered/image_raw", Image)
        self.pub_cloud = pub("depth/points", PointCloud2)
        self.pub_audio = self.create_publisher(Int16MultiArray, "audio", 10)

        self.color_frame = f"{self.frame_prefix}_rgb_optical_frame"
        self.depth_frame = f"{self.frame_prefix}_ir_optical_frame"
        self._static_tf()
        self.depth_params, self.color_params = dict(DEPTH_PARAMS), dict(COLOR_PARAMS)
        self.frames = 0
        self.running = True
        self.audio_stream = None

        if self.simulate:
            self.sim = Simulator()
            self.t0 = time.monotonic()
            self.create_timer(1.0 / self.fps, self._simulate_step)
            if self.audio_enabled:
                self.create_timer(0.1, self._simulate_audio)
        else:
            threading.Thread(target=self._run_freenect2, daemon=True).start()
            if self.audio_enabled:
                self._start_audio()
        geraet = "(simulated)" if self.simulate else self.serial or "(first device)"
        self.get_logger().info(
            f"Kinect v2 {geraet}: colour 1/{self.color_step}, depth + IR 512x424"
        )

    def _static_tf(self):
        """link -> rgb optical frame, link -> ir optical frame (about 5 cm apart)."""
        transforms = []
        for child, y in ((self.color_frame, 0.0), (self.depth_frame, -0.052)):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = f"{self.frame_prefix}_link"
            t.child_frame_id = child
            t.transform.translation.y = y
            t.transform.rotation.x, t.transform.rotation.y = -0.5, 0.5
            t.transform.rotation.z, t.transform.rotation.w = -0.5, 0.5
            transforms.append(t)
        self.tf = StaticTransformBroadcaster(self)
        self.tf.sendTransform(transforms)

    # ------------------------------------------------------------------ publishing
    def _stamp(self, msg, stamp, frame):
        msg.header.stamp, msg.header.frame_id = stamp, frame
        return msg

    def publish_frames(self, color_bgrx, depth_mm_f, ir_f=None, registered_bgrx=None):
        """color_bgrx already thinned out by color_step, depth/IR as float32 512x424."""
        stamp = self.get_clock().now().to_msg()
        rgb = bgrx_to_rgb(color_bgrx)
        c = scaled(self.color_params, 1.0 / self.color_step)
        self.pub_color.publish(
            self._stamp(Image(**image_fields(rgb, "rgb8")), stamp, self.color_frame)
        )
        self.pub_color_info.publish(
            self._stamp(
                CameraInfo(**camera_info_fields(rgb.shape[1], rgb.shape[0], **c)),
                stamp,
                self.color_frame,
            )
        )
        depth = float_to_u16(depth_mm_f)
        dinfo = CameraInfo(
            **camera_info_fields(depth.shape[1], depth.shape[0], **self.depth_params)
        )
        self.pub_depth.publish(
            self._stamp(Image(**image_fields(depth, "16UC1")), stamp, self.depth_frame)
        )
        self.pub_depth_info.publish(self._stamp(dinfo, stamp, self.depth_frame))
        if self.publish_ir and ir_f is not None:
            self.pub_ir.publish(
                self._stamp(
                    Image(**image_fields(float_to_u16(ir_f), "mono16")),
                    stamp,
                    self.depth_frame,
                )
            )
            self.pub_ir_info.publish(self._stamp(dinfo, stamp, self.depth_frame))
        reg = bgrx_to_rgb(registered_bgrx) if registered_bgrx is not None else None
        if self.publish_registered and reg is not None:
            self.pub_registered.publish(
                self._stamp(Image(**image_fields(reg, "rgb8")), stamp, self.depth_frame)
            )
        self.frames += 1
        if (
            self.cloud_every > 0
            and self.frames % self.cloud_every == 0
            and self.pub_cloud.get_subscription_count() > 0
        ):
            pts = points_xyz(depth, self.cloud_step, rgb=reg, **self.depth_params)
            self.pub_cloud.publish(self._cloud(pts, stamp))

    def _cloud(self, pts, stamp):
        fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate("xyz")
        ]
        if pts.shape[1] == 4:
            fields.append(
                PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1)
            )
        msg = PointCloud2(
            height=1,
            width=len(pts),
            fields=fields,
            is_bigendian=False,
            point_step=4 * pts.shape[1],
            row_step=4 * pts.shape[1] * len(pts),
            is_dense=True,
            data=np.ascontiguousarray(pts).tobytes(),
        )
        return self._stamp(msg, stamp, self.depth_frame)

    def publish_audio(self, samples, rate=16000):
        msg = Int16MultiArray(data=samples.ravel().tolist())
        frames, channels = samples.shape if samples.ndim == 2 else (samples.size, 1)
        msg.layout.dim = [
            MultiArrayDimension(label=f"frames@{rate}Hz", size=frames, stride=samples.size),
            MultiArrayDimension(label="channels", size=channels, stride=channels),
        ]
        self.pub_audio.publish(msg)

    # ------------------------------------------------------------------ simulation
    def _simulate_step(self):
        t = time.monotonic() - self.t0
        depth = self.sim.depth(t)
        reg = self.sim.registered(t) if self.publish_registered else None
        self.publish_frames(self.sim.color(t, self.color_step), depth, self.sim.ir(t), reg)

    def _simulate_audio(self):
        self.publish_audio(
            self.sim.audio(time.monotonic() - self.t0, channels=self.audio_channels)
        )

    # ------------------------------------------------------------------ hardware
    def _run_freenect2(self):
        try:
            import pylibfreenect2 as f2
        except ImportError as e:
            self.get_logger().error(f"pylibfreenect2 missing: {e}")
            return
        pipelines = {
            "cpu": "CpuPacketPipeline",
            "opengl": "OpenGLPacketPipeline",
            "opencl": "OpenCLPacketPipeline",
            "cuda": "CudaPacketPipeline",
        }
        try:
            pipeline = getattr(f2, pipelines[self.pipeline])()
        except (AttributeError, KeyError, RuntimeError):
            self.get_logger().warn(f"pipeline {self.pipeline} not available, using cpu")
            pipeline = f2.CpuPacketPipeline()
        fn = f2.Freenect2()
        if fn.enumerateDevices() == 0:
            self.get_logger().error("no Kinect v2 found (USB 3.0 and power supply?)")
            return
        serial = self.serial or fn.getDeviceSerialNumber(0)
        device = fn.openDevice(serial, pipeline=pipeline)
        listener = f2.SyncMultiFrameListener(
            f2.FrameType.Color | f2.FrameType.Ir | f2.FrameType.Depth
        )
        device.setColorFrameListener(listener)
        device.setIrAndDepthFrameListener(listener)
        device.start()
        ir_p, col_p = device.getIrCameraParams(), device.getColorCameraParams()
        self.depth_params = {"fx": ir_p.fx, "fy": ir_p.fy, "cx": ir_p.cx, "cy": ir_p.cy}
        self.color_params = {"fx": col_p.fx, "fy": col_p.fy, "cx": col_p.cx, "cy": col_p.cy}
        registration = f2.Registration(ir_p, col_p)
        undistorted, registered = f2.Frame(512, 424, 4), f2.Frame(512, 424, 4)
        self.get_logger().info(f"Kinect v2 {serial} running ({self.pipeline} pipeline)")
        try:
            while self.running and rclpy.ok():
                frames = listener.waitForNewFrame()
                color, ir, depth = frames["color"], frames["ir"], frames["depth"]
                registration.apply(color, depth, undistorted, registered)
                s = self.color_step
                self.publish_frames(
                    color.asarray()[::s, ::s].copy(),
                    undistorted.asarray(np.float32).copy(),
                    ir.asarray().copy(),
                    registered.asarray(np.uint8).copy(),
                )
                listener.release(frames)
        finally:
            device.stop()
            device.close()

    def _start_audio(self):
        """The Kinect v2 microphone array is a standard USB audio device (4 channels)."""
        try:
            import sounddevice as sd

            device = (
                int(self.audio_device)
                if self.audio_device.isdigit()
                else (self.audio_device or None)
            )
            self.audio_stream = sd.InputStream(
                device=device,
                samplerate=16000,
                channels=self.audio_channels,
                dtype="int16",
                blocksize=1600,
                callback=lambda data, *_: self.publish_audio(data.copy()),
            )
            self.audio_stream.start()
        except Exception as e:
            self.get_logger().warn(f"audio disabled: {e}")

    def destroy_node(self):
        self.running = False
        if self.audio_stream is not None:
            self.audio_stream.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KinectV2Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
