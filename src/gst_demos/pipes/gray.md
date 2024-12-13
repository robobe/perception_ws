## h264 send
```
gst-launch-1.0  \
    videotestsrc ! videoconvert  \
    ! x264enc  bitrate=2000 tune=zerolatency speed-preset=veryslow qp-min=0 qp-max=0 \
    ! rtph264pay  \
    ! udpsink host=127.0.0.1 port=5000

```

```
gst-launch-1.0  \
    videotestsrc ! videoconvert  \
    ! video/x-raw,format=GRAY8 \
    ! videoconvert \
    ! video/x-raw,format=I420 \
    ! videoconvert \
    ! x264enc  bitrate=2000 tune=zerolatency speed-preset=veryslow qp-min=0 qp-max=0 \
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