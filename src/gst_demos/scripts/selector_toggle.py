import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initialize GStreamer
Gst.init(None)

# Create the pipeline
pipeline = Gst.parse_launch("""
    input-selector name=selector 
    videotestsrc pattern=0 ! video/x-raw,framerate=30/1 ! queue ! selector.sink_0 
    videotestsrc pattern=1 ! video/x-raw,framerate=30/1 ! queue ! selector.sink_1 
    selector. ! autovideosink
""")

# Get the input-selector element
selector = pipeline.get_by_name("selector")

# Start the pipeline
pipeline.set_state(Gst.State.PLAYING)
active_sink = "sink_1"

# Switch pads dynamically
def switch_source():
    global active_sink
    # Get the pad to activate
    if active_sink== "sink_0":
        
        active_sink="sink_1"
    else:
        active_sink="sink_0"

    sink_1_pad = selector.get_static_pad(active_sink)
    selector.set_property("active-pad", sink_1_pad)
    print("Switched to source 1")
    GLib.timeout_add_seconds(2, switch_source)

# Schedule a pad switch after 5 seconds
GLib.timeout_add_seconds(2, switch_source)

# Run the main loop
try:
    loop = GLib.MainLoop()
    loop.run()
except KeyboardInterrupt:
    pipeline.set_state(Gst.State.NULL)