import gi
from fastapi import FastAPI
from gi.repository import Gst, GLib
import threading
import time
import asyncio
from starlette.responses import StreamingResponse

# Initialize GStreamer
Gst.init(None)

# Global variables to store the pipeline and bus
pipeline = None
bus = None

# FastAPI app
app = FastAPI()

# Function to update the GStreamer pipeline with new bitrate
def update_pipeline(bitrate,qp):
    encoder = pipeline.get_by_name("x264enc")
    if encoder:
        # Set the new bitrate
        encoder.set_property("bitrate", bitrate)
        encoder.set_property("quantizer", int(qp))

def update_pipeline_rate(rate):
    videorate = pipeline.get_by_name("rate")
    capsfilter = pipeline.get_by_name("capsfilter")
    # videorate.get_static_pad("src").set_caps(Gst.Caps.from_string(f"video/x-raw,framerate={rate}/1"))
    capsfilter.set_property("caps", Gst.Caps.from_string(f"video/x-raw,framerate={rate}/1"))

    print(rate)

# Function to create and start the pipeline
def start_pipeline():
    global pipeline
    # Default pipeline description
    # stream must be sync to keep rate (fps) don't add sync false to udpsink
    pipeline_description = """videotestsrc ! video/x-raw, width=640, height=480, framerate=30/1, format=GRAY8 ! videoconvert \
        ! videorate name=rate \
        ! capsfilter name=capsfilter caps=video/x-raw,framerate=10/1 \
        ! x264enc name=x264enc bitrate=1000 quantizer=10 \
        ! rtph264pay ! udpsink host=127.0.0.1 port=5000"""
    pipeline = Gst.parse_launch(pipeline_description)

    # Get the bus to listen for errors
    global bus
    bus = pipeline.get_bus()

    # Start the pipeline
    pipeline.set_state(Gst.State.PLAYING)



# Start the GStreamer pipeline in a separate thread
def start_gstreamer_thread():
    threading.Thread(target=start_pipeline, daemon=True).start()


@app.post("/update-rate/{rate}")
async def update_rate(rate: int):
    update_pipeline_rate(rate)
    return {"message": f"rate updated to {rate} fps"}

# Endpoint to update bitrate dynamically
@app.post("/update-bitrate/{bitrate}")
async def update_bitrate(bitrate: int, qp):
    # Update the pipeline with the new bitrate
    """
    High quality, High qp,qr: 40
    low, qp:10 qr:5
    """
    update_pipeline(bitrate, qp )
    return {"message": f"Bitrate updated to {bitrate} kbps"}

# Start the server with FastAPI and Uvicorn
if __name__ == "__main__":
    # Start GStreamer in a background thread
    start_gstreamer_thread()

    # Run the FastAPI app using Uvicorn
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
