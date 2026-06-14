#!/usr/bin/env python3
"""
Nodo ROS2 — Receiver UDP H264 → /image_raw
Recibe stream RTP H264 por UDP y publica sensor_msgs/Image sin cv_bridge.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import numpy as np


def ndarray_to_imgmsg(frame: np.ndarray, frame_id: str, stamp) -> Image:
    msg = Image()
    msg.header.stamp    = stamp
    msg.header.frame_id = frame_id
    msg.height          = frame.shape[0]
    msg.width           = frame.shape[1]
    msg.encoding        = "bgr8"
    msg.is_bigendian    = False
    msg.step            = frame.shape[1] * 3
    msg.data            = frame.tobytes()
    return msg


class GStreamerBridgeNode(Node):
    def __init__(self):
        super().__init__('gstreamer_bridge_node')

        self.declare_parameter("udp_port",    5001)
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("frame_id",    "camera")

        port      = self.get_parameter("udp_port").value
        topic     = self.get_parameter("image_topic").value
        self.fid  = self.get_parameter("frame_id").value

        self.publisher = self.create_publisher(Image, topic, 10)

        Gst.init(None)

        self.pipeline = Gst.parse_launch(
            f'udpsrc port={port} '
            f'caps="application/x-rtp, media=video, encoding-name=H264, payload=96" ! '
            f'rtpjitterbuffer ! rtph264depay ! avdec_h264 ! '
            f'videoconvert ! video/x-raw,format=BGR ! '
            f'appsink name=sink emit-signals=true drop=true max-buffers=1'
        )

        self.appsink = self.pipeline.get_by_name("sink")
        self.appsink.set_property("emit-signals", True)
        self.appsink.set_property("drop", True)
        self.appsink.set_property("max-buffers", 1)
        self.appsink.connect("new-sample", self.on_new_sample)

        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(f"Receiver escuchando en UDP:{port} → {topic}")

    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK

        buf  = sample.get_buffer()
        caps = sample.get_caps()
        frame = self._buf_to_array(buf, caps)

        if frame is not None:
            stamp = self.get_clock().now().to_msg()
            self.publisher.publish(ndarray_to_imgmsg(frame, self.fid, stamp))

        return Gst.FlowReturn.OK

    def _buf_to_array(self, buf, caps) -> np.ndarray | None:
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return None
        try:
            w = caps.get_structure(0).get_value('width')
            h = caps.get_structure(0).get_value('height')
            data = np.frombuffer(map_info.data, np.uint8)
            return data.reshape((h, w, 3)).copy()
        except Exception as e:
            self.get_logger().warn(f"Error convirtiendo frame: {e}")
            return None
        finally:
            buf.unmap(map_info)

    def destroy_node(self):
        self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GStreamerBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()