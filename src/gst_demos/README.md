# Video Toggle

Ros2 node that wrap gstreamer pipe the toggle between multiple inputs

```
xhost +local:
```



```
docker run -it --rm \
--user user \
--net host \
--name test \
--hostname test \
--env DISPLAY=$DISPLAY \
--env XDG_RUNTIME_DIR=/tmp/xdg \
--volume="$HOME/.Xauthority:$HOME/.Xauthority:rw" \
-v `pwd`:/tmp/install \
vsc-rome_ws-00e16b1bdb04c0d1ea892a0568e41976c49638b0eedd1ee7b73eeccb03b12198 \
/bin/bash
```

# deploy
- Run docker
- Run `apt update`
- Install `sudo apt install gst_demos-1.2.3-Linux.deb`

```
docker run -it --rm \
--user user \
--net host \
--name test \
--hostname test \
--env="DISPLAY" \
--volume="$HOME/.Xauthority:$HOME/.Xauthority:rw" \
-v `pwd`:/tmp/install \
humble.test \
/bin/bash
```

```bash
gst-launch-1.0 videotestsrc ! videoconvert ! autovideosink
```

```bash
ros2 run gst_demos toggle_source.py
```

!!! note "gstreamer"
    ???
    ```
    export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libunwind.so.8
    ```