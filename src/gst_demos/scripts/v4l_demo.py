import sys
import numpy as np
import traceback
import time
import gi
import cv2
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initializes Gstreamer, it's variables, paths
Gst.init(sys.argv)

# PIPELINE = "appsrc name=app_src  is-live=true format=GST_FORMAT_TIME ! video/x-raw,width=640,height=480,format=BGR,framerate=30/1 ! videoconvert ! queue max-size-buffers=1 leaky=downstream ! videorate ! video/x-raw, framerate=10/1 ! fpsdisplaysink"
PIPELINE = "v4l2src name=x device=/dev/video4 ! video/x-raw,width=640,height=512,framerate=9/1 ! videoconvert ! textoverlay name=txt text='counter: 0' valignment=top  ! videorate ! video/x-raw, framerate=2/1 ! fpsdisplaysink"

def ndarray_to_gst_buffer(array: np.ndarray) -> Gst.Buffer:
    """Converts numpy array to Gst.Buffer"""
    
    return Gst.Buffer.new_wrapped(array.tobytes())


def on_message(bus: Gst.Bus, message: Gst.Message, loop: GLib.MainLoop):
    mtype = message.type
    if mtype == Gst.MessageType.EOS:
        print("End of stream")
        loop.quit()

    elif mtype == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print(err, debug)
        loop.quit()

    elif mtype == Gst.MessageType.WARNING:
        err, debug = message.parse_warning()
        print(err, debug)

    return True

def generate_frame(counter):
    """Generate a simple image using OpenCV."""
    # Create a blank RGB image
    height, width = 480, 640
    frame = np.zeros((height, width, 3), dtype=np.uint8)

    # Draw a moving circle
    x = counter % width
    center = (x, height // 2)
    cv2.circle(frame, center, 50, (0, 255, 0), -1)  # Green circle

    # Define text properties
    text = str(counter)
    org = (50, 100)  # Text position (x, y)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    color = (0, 255, 0)  # Green color in BGR
    thickness = 2

    # Add text to the image
    cv2.putText(frame, text, org, font, font_scale, color, thickness, cv2.LINE_AA)

    return frame

pipeline = Gst.parse_launch(PIPELINE)
appsource = pipeline.get_by_name("app_src")
bus = pipeline.get_bus()
# allow bus to emit messages to main thread
bus.add_signal_watch()
# Start pipeline
pipeline.set_state(Gst.State.PLAYING)
# Init GObject loop to handle Gstreamer Bus Events
loop = GLib.MainLoop()

# Add handler to specific signal
bus.connect("message", on_message, loop)
TARGET_FPS = 10
frame_interval = 1 / TARGET_FPS
global last_timestamp
last_timestamp = time.time()  # Track last timestamp
txt = pipeline.get_by_name("txt")
# Push buffer and check
for i in range(100):
    txt.set_property("text", str(i))
    time.sleep(1/9)
#     # arr = np.random.randint(low=0,high=255,size=(480,640,3),dtype=np.uint8)
#     arr = generate_frame(i)
#     # Get current time
#     current_time = time.time()

#     # Skip frames if the time difference between the current and last timestamp is too small
    
#     time_diff = current_time - last_timestamp
    
#     # if time_diff > frame_interval:
#         # Skip this frame as it's too old
#     last_timestamp = current_time
#     data = ndarray_to_gst_buffer(arr)
#     appsource.emit("push-buffer", data)
#     time.sleep(1/30)
    
# appsource.emit("end-of-stream")

try:
    loop.run()
except Exception:
    traceback.print_exc()
    loop.quit()

# Stop Pipeline
pipeline.set_state(Gst.State.NULL)