# Pipes

## selector
```bash
gst-launch-1.0   input-selector name=selector   \
videotestsrc pattern=0 ! video/x-raw,framerate=30/1 ! queue ! selector.sink_0   \
videotestsrc pattern=1 ! video/x-raw,framerate=30/1 ! queue ! selector.sink_1   \
selector. \
! autovideosink

```
## build
```
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/ros/humble
make
make package
```

## docker for test
- create docker with x11-apps
- check gui ok with xeyes for example
- test app

## Test
run `FROM humble:dev` docker image

```
docker run -it \
  --rm \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged \
  --name test \
  --net host \
  --hostname test \
  -v `pwd`:/tmp/install \
  vsc-rome_ws-00e16b1bdb04c0d1ea892a0568e41976c49638b0eedd1ee7b73eeccb03b12198:latest \
  bash
```

vsc-rome_ws-00e16b1bdb04c0d1ea892a0568e41976c49638b0eedd1ee7b73eeccb03b12198:latest

```
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libunwind.so.8
```

# TODO
- run docker with gui
- check package 
- don't forget to install with apt
- 