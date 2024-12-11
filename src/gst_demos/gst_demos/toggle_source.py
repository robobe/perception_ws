#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import gi
gi.require_version('Gst', '1.0') 
from gi.repository import Gst, GLib
import threading
from gst_demos.srv import StreamRequest
import traceback

class GStreamerNode(Node):
    def __init__(self):
        super().__init__('gstreamer_node')
        self.get_logger().info("Starting GStreamer Node...")

        # Initialize GStreamer
        Gst.init(None)

        # Create the pipeline
        # TODO: get pipe from outside
        self.pipeline = Gst.parse_launch("""
            input-selector name=selector 
            videotestsrc pattern=0 ! video/x-raw,framerate=30/1 ! queue ! selector.sink_0 
            videotestsrc pattern=1 ! video/x-raw,framerate=30/1 ! queue ! selector.sink_1 
            selector. ! autovideosink
        """)
        self.selector = self.pipeline.get_by_name("selector")

        # Start the pipeline
        self.pipeline.set_state(Gst.State.PLAYING)

        # Timer to switch sources every 5 seconds
        # self.switch_timer = self.create_timer(2.0, self.switch_source)
        self.switch_srv = self.create_service(StreamRequest, "switch_stream", self.__switch_stream_handler)
        # GStreamer main loop in a separate thread
        self.mainloop = GLib.MainLoop()
        self.mainloop_thread = threading.Thread(target=self.mainloop.run)
        self.mainloop_thread.start()

    def __switch_stream_handler(self, request: StreamRequest.Request, response: StreamRequest.Response):
        """
        set selector active stream

        ros2 service call /switch_stream gst_demos/srv/StreamRequest "{number: 3}"
        """
        #TODO: move mapping to parameters
        stream_mapping = {
            1: "sink_0",
            2: "sink_1"
        }
        # TODO: check current and not switch if request current
        if request.number not in stream_mapping:
            response.success = False
            response.msg = "stream id not found"
            return response
        try:
            pad_name = stream_mapping[request.number]
            request_pad = self.selector.get_static_pad(pad_name)
            self.selector.set_property("active-pad", request_pad)
            response.success = True
        except:
            response.success = False
            response.msg = "gst fail to switch stream"
            self.get_logger().error(f"Fail to switch stream: \n{traceback.format_exc()}")
        finally:
            return response

    def switch_source(self):
        """Switch between input sources."""
        current_pad = self.selector.get_property("active-pad")
        sink_0_pad = self.selector.get_static_pad("sink_0")
        sink_1_pad = self.selector.get_static_pad("sink_1")

        # Switch to the other pad
        new_pad = sink_0_pad if current_pad == sink_1_pad else sink_1_pad
        self.selector.set_property("active-pad", new_pad)
        self.get_logger().info(f"Switched to {'source 0' if new_pad == sink_0_pad else 'source 1'}")

    def destroy_node(self):
        """Clean up resources when shutting down the node."""
        self.get_logger().info("Shutting down GStreamer pipeline...")
        self.pipeline.set_state(Gst.State.NULL)
        self.mainloop.quit()
        self.mainloop_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GStreamerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
