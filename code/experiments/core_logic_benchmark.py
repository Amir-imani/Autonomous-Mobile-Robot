"""Direct hardware-free checks for protocol, odometry and LiDAR source logic."""
from __future__ import annotations
import argparse,math,os,struct,time
from dataclasses import asdict,dataclass
from pathlib import Path
from common import describe,environment_metadata,json_safe,write_csv,write_json
from resource_probe import ResourceSampler
from source_harness import load_lidar_symbols,load_odometry_symbols,load_serial_symbols

@dataclass
class Check: name:str; passed:bool; detail:str; result_type:str="behavior_verification"
def main():
    p=argparse.ArgumentParser(); p.add_argument("--output",type=Path,required=True); p.add_argument("--trials",type=int,default=3); p.add_argument("--iterations",type=int,default=5000); a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=True)
    sampler=ResourceSampler(os.getpid()); sampler.start(); checks=[]; raw=[]
    serial=load_serial_symbols(); crc=serial["crc16_ibm"]; Parser=serial["RobotStateParser"]; state_struct=serial["ROBOT_STATE_STRUCT"]; header=serial["FRAME_HEADER"]
    payload=state_struct.pack(123,*([0.0]*14),0,0,0,0) if state_struct.size==76 else bytes(state_struct.size)
    frame=header+payload+struct.pack("<H",crc(payload)); parser=Parser()
    parsed=[]
    for b in frame: parsed.extend(parser.feed(bytes([b])))
    checks.append(Check("serial_frame_roundtrip",len(parsed)==1,f"frames={len(parsed)}"))
    checks.append(Check("crc_rejects_corruption",crc(payload)!=crc(payload[:-1]+bytes([payload[-1]^1])),"single-byte mutation changes CRC"))
    K=load_odometry_symbols()["DifferentialDriveKinematics"]; model=K(0.03335,0.28); v,w=model.update(30,30,1.0)
    checks.append(Check("straight_odometry",abs(w)<1e-12 and model.x>0 and abs(model.y)<1e-12,f"x={model.x:.6f}"))
    lidar=load_lidar_symbols(); C=lidar["ScanConfiguration"]; Acc=lidar["LidarSweepAccumulator"]; Sample=lidar["LidarSample"]
    c=C("laser_link",0.0,180.0,90.0,1.0,0.001,0.02,4.0,0.25,1.0,2.0,1.0,3); acc=Acc(c); completed=None
    for i,ang in enumerate([0,30,60,90,120,150,180,175,170]):
        out=acc.consume(Sample(float(ang),1.0,True,i*20_000_000))
        if out is not None: completed=out
    checks.append(Check("lidar_reversal_scan",completed is not None,"direction reversal completes a sweep"))
    for name,fn in [("parser",lambda: [Parser().feed(frame) for _ in range(a.iterations)]),("odometry",lambda:[K(0.03335,0.28).update(30,29,0.02) for _ in range(a.iterations)])]:
        for trial in range(1,a.trials+1):
            t=time.perf_counter_ns(); fn(); elapsed=time.perf_counter_ns()-t; raw.append({"benchmark":name,"trial":trial,"iterations":a.iterations,"total_ns":elapsed,"ns_per_operation":elapsed/a.iterations,"throughput_hz":a.iterations*1e9/elapsed})
    resources=sampler.stop(); write_csv(a.output/"core_checks.csv",[asdict(x) for x in checks]); write_csv(a.output/"core_microbenchmarks.csv",raw)
    write_json(a.output/"core_summary.json",json_safe({"environment":environment_metadata(),"checks":{"passed":sum(x.passed for x in checks),"failed":sum(not x.passed for x in checks),"behavior_verifications_passed":sum(x.passed for x in checks),"detected_defects":0,"items":[asdict(x) for x in checks]},"process_resources":resources}))
    return 0 if all(x.passed for x in checks) else 1
if __name__=="__main__": raise SystemExit(main())
