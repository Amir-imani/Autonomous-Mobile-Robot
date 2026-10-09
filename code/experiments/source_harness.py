"""Load hardware-independent classes directly from source files."""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass
from enum import Enum,IntEnum,auto
import math
from pathlib import Path
import struct,threading,time
from typing import Deque,Dict,List,Optional,Sequence,Set,Tuple

ROOT=Path(__file__).resolve().parents[1]
ROS_PACKAGE=ROOT/"ros2"/"pico_serial_bridge"/"pico_serial_bridge"

def _execute_slice(path:Path,start:str,end:str)->dict[str,object]:
    source=path.read_text(encoding="utf-8"); a=source.index(start); b=source.index(end,a)
    namespace={"__builtins__":__builtins__,"deque":deque,"dataclass":dataclass,"Enum":Enum,"IntEnum":IntEnum,"auto":auto,"math":math,"struct":struct,"threading":threading,"time":time,"Deque":Deque,"Dict":Dict,"List":List,"Optional":Optional,"Sequence":Sequence,"Set":Set,"Tuple":Tuple}
    exec(compile(source[a:b],str(path),"exec"),namespace); return namespace

def load_serial_symbols()->dict[str,object]:
    return _execute_slice(ROS_PACKAGE/"serial_bridge_node.py","FRAME_HEADER =","class PicoSerialBridge")
def load_odometry_symbols()->dict[str,object]:
    return _execute_slice(ROS_PACKAGE/"differential_odometry_node.py","class DifferentialDriveKinematics","class DifferentialOdometryNode")
def load_lidar_symbols()->dict[str,object]:
    return _execute_slice(ROS_PACKAGE/"lidar_scan_node.py","class SweepDirection","class LidarScanNode")
