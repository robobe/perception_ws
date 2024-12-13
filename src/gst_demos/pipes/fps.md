
## test
```
gst-launch-1.0 videotestsrc \
! fpsdisplaysink
```

## test with rate
```
gst-launch-1.0 videotestsrc \
! videorate ! "video/x-raw, framerate=10/1" \
! fpsdisplaysink 
```

## appsrc

```
appsrc name=app_src  is-live=true format=GST_FORMAT_TIME \
! video/x-raw,width=640,height=480,format=BGR,framerate=30/1 \
! videoconvert \
! queue max-size-buffers=1 leaky=downstream \
! videorate \
! video/x-raw, framerate=10/1 \
! fpsdisplaysink"
```
The queue element in GStreamer is a crucial component for buffering data between different parts of a pipeline.

- **max-size-buffers**: Maximum number of buffers the queue can hold.
- **leaky**: Controls the behavior of the queue when it is full.

- **none** (default): The queue blocks upstream elements when it is full. No buffers are dropped, ensuring no data is lost, but this can introduce latency.
- **upstream**: Discards the oldest buffer in the queue to make space for new buffers. This prevents blocking upstream elements and is useful for live sources or real-time streaming.
- **downstream**: Discards the newest buffer when the queue is full. This ensures downstream elements only process the oldest data.

## capture camera
```
gst-launch-1.0 v4l2src device=/dev/video4 \
! videoconvert \
! fpsdisplaysink 
```

## capture camera with rate
```
gst-launch-1.0 v4l2src device=/dev/video4 \
! videoconvert \
! videorate ! "video/x-raw, framerate=4/1" \
! fpsdisplaysink 
```


## h264 send
```
gst-launch-1.0  \
    v4l2src device=/dev/video4 ! videoconvert \
    ! x264enc tune=zerolatency \
    ! rtph264pay \
    ! udpsink host=127.0.0.1 port=5000

```

## h264 recv
```
gst-launch-1.0 -v \
    udpsrc port=5000 caps="application/x-rtp, media=video, encoding-name=H264, payload=96" \
    ! rtph264depay \
    ! avdec_h264 \
    ! videoconvert \
    ! fpsdisplaysink

```

## h264 with rate
```
gst-launch-1.0 -v \
    v4l2src device=/dev/video4 ! videoconvert \
    ! videorate ! "video/x-raw, framerate=2/1" \
    ! x264enc tune=zerolatency \
    ! rtph264pay \
    ! udpsink host=127.0.0.1 port=5000

```

```
gst-launch-1.0 -v \
    udpsrc port=5000 caps="application/x-rtp, media=video, encoding-name=H264, payload=96" \
    ! rtph264depay \
    ! avdec_h264 \
    ! videoconvert \
    ! fpsdisplaysink

```


## h265 with rate
```
gst-launch-1.0 -v \
    v4l2src device=/dev/video4 ! videoconvert \
    ! videorate ! "video/x-raw, framerate=2/1" \
    ! x265enc tune=zerolatency \
    ! rtph265pay \
    ! udpsink host=127.0.0.1 port=5000

```

```
gst-launch-1.0 -v \
    v4l2src device=/dev/video4 ! videoconvert \
    ! x265enc tune=zerolatency \
    ! rtph265pay \
    ! udpsink host=127.0.0.1 port=5000 sync=

```

```
gst-launch-1.0 -v \
    udpsrc port=5000 caps="application/x-rtp, media=video, encoding-name=H265, payload=96" \
    ! rtph265depay \
    ! avdec_h265 \
    ! videoconvert \
    ! fpsdisplaysink

```
