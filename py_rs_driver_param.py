from enum import IntEnum
from dataclasses import dataclass


@dataclass
class LidarType(IntEnum):  # < LiDAR type
    # mechanical
    RS16 = 0x01
    RS32 = 0x02
    RSBP = 0x03
    RS128 = 0x04
    RS80 = 0x05
    RS48 = 0x06
    RSP128 = 0x07
    RSP80 = 0x08
    RSP48 = 0x09
    RSHELIOS = 0x0A
    RSHELIOS_16P = 0x0B
    RSROCK = 0x0C

    # mems
    RSM1 = 0x20
    RSM2 = 0x21
    RSEOS = 0x22

    # jumbo
    RS_JUMBO = 0x100
    RSM1_JUMBO = RS_JUMBO + RSM1


@dataclass
class InputType(IntEnum):
    ONLINE_LIDAR = 1
    PCAP_FILE = 2
    RAW_PACKET = 3


@dataclass
class SplitFrameMode(IntEnum):
    SPLIT_BY_ANGLE = 1
    SPLIT_BY_FIXED_BLKS = 2
    SPLIT_BY_CUSTOM_BLKS = 3


@dataclass
class RSTransformParam:  # < The Point transform parameter
    x: float = 0.0  # < unit, m
    y: float = 0.0  # < unit, m
    z: float = 0.0  # < unit, m
    roll: float = 0.0  # < unit, radian
    pitch: float = 0.0  # < unit, radian
    yaw: float = 0.0  # < unit, radian


@dataclass
class RSDecoderParam:  # < LiDAR decoder parameter
    config_from_file: bool = False  # < Internal use only for debugging
    angle_path: str = ""  # < Internal use only for debugging
    wait_for_difop: bool = True  # < true: start sending point cloud until receive difop packet
    min_distance: float = 0.2  # < Minimum distance of point cloud range
    max_distance: float = 200.0  # < Max distance of point cloud range
    start_angle: float = 0.0  # < Start angle of point cloud
    end_angle: float = 360.0  # < End angle of point cloud
    split_frame_mode: SplitFrameMode = SplitFrameMode.SPLIT_BY_ANGLE
    # < 1: Split frames by split_angle;
    # < 2: Split frames by fixed number of blocks;
    # < 3: Split frames by custom number of blocks (num_blks_split)
    split_angle: float = 0.0  # < Split angle(degree) used to split frame, only be used when split_frame_mode=1
    num_blks_split: int = 1  # < Number of packets in one frame, only be used when split_frame_mode=3
    use_lidar_clock: bool = False  # < true: use LiDAR clock as timestamp; false: use system clock as timestamp
    dense_points: bool = False  # < true: discard NAN points; false: reserve NAN points
    ts_first_point: bool = False  # < true: time-stamp point cloud with the first point; false: with the last point;
    transform_param: RSTransformParam = RSTransformParam()  # < Used to transform points


@dataclass
class RSInputParam:  # < The LiDAR input parameter
    msop_port: int = 6699  # < Msop packet port number
    difop_port: int = 7788  # < Difop packet port number
    host_address: str = "0.0.0.0"  # < Address of host
    group_address: str = "0.0.0.0"  # < Address of multicast group
    pcap_path: str = ""  # < Absolute path of pcap file
    pcap_repeat: bool = True
    # < true: The pcap bag will repeat play
    pcap_rate: float = 1.0  # < Rate to read the pcap file
    use_vlan: bool = False  # < Vlan on-off
    user_layer_bytes: int = 0  # < Bytes of user layer. thers is no user layer if it is 0
    tail_layer_bytes: int = 0  # < Bytes of tail layer. thers is no tail layer if it is 0


@dataclass
class RSDriverParam:  # < The LiDAR driver parameter
    lidar_type: LidarType = LidarType.RSM1  # < Lidar type
    input_type: InputType = InputType.PCAP_FILE  # < Input type
    input_param: RSInputParam = RSInputParam()  # < Input parameter
    decoder_param: RSDecoderParam = RSDecoderParam()  # < Decoder parameter
