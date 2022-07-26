# py_rs_driver

**Unofficial** python wrapper for [RoboSense rs_driver](https://github.com/RoboSense-LiDAR/rs_driver).

### Environment

- Ubuntu 20.04
- Python 3.8
- rs_driver (v1.5.4 or higher)

The wrapper works correctly on Ubunt20.04 and RS-LiDAR-M1 data. But, this does not mean "perfect"...

### Usage 

#### 1. Install rs_driver(v1.5.4 or higher)

#### 2. Build wrapper library

```bash
$ git clone https://github.com/nobu-e758/py_rs_driver
$ cd py_rs_driver
$ mkdir native/build && cd native/build
$ cmake .. && make
$ ls rs_driver_wrapper.cpython-38-x86_64-linux-gnu.so
```

#### 3. Run sample

```bash
$ cd ../../
$ export PYTHONPATH=./native/build
$ (save .pcap file to ./_input/lidar.pcap)
$ python3.8 main.py
```

### TODO

These methods are not yet implemented.

- LidarDriver::regPacketCallback(...)
- LidarDriver::decodePacket(...)
