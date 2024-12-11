
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
