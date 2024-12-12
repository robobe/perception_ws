# Hammer

Kill nodes by id or name
Expose two interfaces
- ROS Service
- Web REST


### Usage
Run node with web service

```bash
ros2 run hammer hammer.py --web
```

### Pack

```bash
colcon build
cd build/hammer
cpack
```