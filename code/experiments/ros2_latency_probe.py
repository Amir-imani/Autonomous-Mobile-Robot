"""ROS 2 Humble CompressedImage latency/rate/drop probe."""
from __future__ import annotations
import argparse,csv
from pathlib import Path
import random,re,threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy,HistoryPolicy,QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
SEQUENCE_PATTERN=re.compile(r"^benchmark:(\d+)$")
def image_qos()->QoSProfile:
    return QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
class ProbePublisher(Node):
    def __init__(self,topic:str,rate_hz:float,payload_bytes:int)->None:
        super().__init__("benchmark_image_publisher"); self.publisher=self.create_publisher(CompressedImage,topic,image_qos()); self.payload=random.Random(20260905).randbytes(payload_bytes); self.sequence=0; self.timer=self.create_timer(1.0/rate_hz,self.publish_frame)
    def publish_frame(self)->None:
        m=CompressedImage(); m.header.stamp=self.get_clock().now().to_msg(); m.header.frame_id=f"benchmark:{self.sequence}"; m.format="synthetic/octet-stream"; m.data=self.payload; self.publisher.publish(m); self.sequence=(self.sequence+1)&0xffffffff
class ProbeSubscriber(Node):
    def __init__(self,topic:str,output:Path)->None:
        super().__init__("benchmark_image_subscriber"); self.output=output; self.rows=[]; self.previous_sequence=None; self.previous_arrival_ns=None; self.subscription=self.create_subscription(CompressedImage,topic,self.callback,image_qos())
    def callback(self,m:CompressedImage)->None:
        arrival=self.get_clock().now().nanoseconds; stamp=int(m.header.stamp.sec)*1_000_000_000+int(m.header.stamp.nanosec); match=SEQUENCE_PATTERN.match(m.header.frame_id); seq=int(match.group(1)) if match else -1
        dropped=0 if self.previous_sequence is None or seq<0 else max(0,seq-self.previous_sequence-1)
        inter="" if self.previous_arrival_ns is None else (arrival-self.previous_arrival_ns)/1e6
        self.rows.append({"arrival_monotonic_ros_ns":arrival,"sequence":seq,"size_bytes":len(m.data),"latency_ms":(arrival-stamp)/1e6,"interarrival_ms":inter,"sequence_gap_drops":dropped,"evidence_class":"A_or_C_ros2_execution_depending_on_hosts"})
        self.previous_sequence=seq; self.previous_arrival_ns=arrival
    def close(self)->None:
        self.output.parent.mkdir(parents=True,exist_ok=True)
        with self.output.open("w",encoding="utf-8",newline="") as s:
            fields=list(self.rows[0]) if self.rows else ["sequence"]; w=csv.DictWriter(s,fieldnames=fields); w.writeheader(); w.writerows(self.rows)
def main()->int:
    p=argparse.ArgumentParser(); p.add_argument("--role",choices=("publisher","subscriber","both"),default="both"); p.add_argument("--topic",default="/benchmark/image/compressed"); p.add_argument("--rate",type=float,default=30.0); p.add_argument("--payload-bytes",type=int,default=153600); p.add_argument("--duration",type=float,default=60.0); p.add_argument("--output",type=Path,default=Path("ros2_latency.csv")); args,ros_args=p.parse_known_args()
    rclpy.init(args=ros_args); nodes=[]; subscriber=None
    if args.role in ("publisher","both"):nodes.append(ProbePublisher(args.topic,args.rate,args.payload_bytes))
    if args.role in ("subscriber","both"): subscriber=ProbeSubscriber(args.topic,args.output); nodes.append(subscriber)
    executor=MultiThreadedExecutor(num_threads=max(2,len(nodes))); [executor.add_node(n) for n in nodes]; stopper=threading.Timer(args.duration,executor.shutdown); stopper.start()
    try:executor.spin()
    finally:
        stopper.cancel()
        if subscriber is not None: subscriber.close()
        for n in nodes:n.destroy_node()
        rclpy.shutdown()
    return 0
if __name__=="__main__":raise SystemExit(main())
