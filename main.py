import numpy as np
import time
import os
import logging
from typing import Sequence

from py_rs_driver_param import *
from rs_driver_wrapper import PyRSDriver

logger = logging.getLogger(__name__)


def pointcloud_callback(
    seq: int, height: int, width: int, timestamp: float, pcd_data: Sequence[float]
):
    """Pointcloud callback

    Args:
        seq: Sequence number of message
        height: Height of point cloud
        width: Width of point cloud
        timestamp: Frame timestamp
        data: Point cloud data.
            2-d double array
            [[x0,y0,z0,i0], [x1,y1,z1,i1], ...]
    """
    np_pcd_data = np.array(pcd_data)
    logger.info(f"Pcd data\t{seq}\t{timestamp}\t{np_pcd_data.shape}\t{np_pcd_data.dtype}")


def exception_callback(error_type: int, error_str: str):
    """Exception calback

    Args:
        error_type: Error type.
            0: Info
            1: Warning
            2: Error
        error_str: Error message.
    """
    if error_type == 0:
        logger.info(error_str)
    elif error_type == 1:
        logger.warn(error_str)
    else:
        logger.error(error_str)


if __name__ == "__main__":

    logging.basicConfig(level=logging.INFO)

    # Configure RSDRiverParam
    param = RSDriverParam()
    param.lidar_type = LidarType.RSM1
    param.input_type = InputType.PCAP_FILE
    param.input_param.pcap_path = os.path.abspath(
        "./_input/lidar.pcap"
    )
    param.decoder_param.use_lidar_clock = True
    param.decoder_param.dense_points = True

    # Instatiate PyRSDriver
    lidar_driver = PyRSDriver()
    lidar_driver.regPointCloudCallback(pointcloud_callback)
    lidar_driver.regExceptionCallback(exception_callback)
    lidar_driver.displayNativeMsg(False)

    lidar_driver.init(py_param=param)
    lidar_driver.start()
    time.sleep(1)
    lidar_driver.stop()
