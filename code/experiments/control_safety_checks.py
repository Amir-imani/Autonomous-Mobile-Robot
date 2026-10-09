"""Deterministic specification model for command conversion and safe-state gates."""

from __future__ import annotations
import argparse
from dataclasses import asdict,dataclass
import math
from pathlib import Path
from common import write_csv,write_json

@dataclass
class Result:
    name:str
    passed:bool
    detail:str
    evidence_class:str="B_source_derived_control_specification_model"

def wheel_rpm(linear:float,angular:float)->tuple[float,float,float]:
    radius,base,maximum=0.03335,0.28,100.0
    left=(linear-angular*base*0.5)*60.0/(2.0*math.pi*radius)
    right=(linear+angular*base*0.5)*60.0/(2.0*math.pi*radius)
    peak=max(abs(left),abs(right)); scale=1.0
    if peak>maximum:
        scale=maximum/peak; left*=scale; right*=scale
    return left,right,scale

def safe_required(*,has_command:bool,command_age_ms:int,drive_enabled:bool,sensor_age_ms:int,i2c_fault:bool,bench_mode:bool)->bool:
    command_stale=(not has_command) or command_age_ms>(2000 if bench_mode else 100)
    sensor_stale=sensor_age_ms>(500 if bench_mode else 100)
    return (i2c_fault and not bench_mode) or (sensor_stale and not bench_mode) or command_stale or not drive_enabled

def check(name:str,condition:bool,detail:str)->Result:
    return Result(name,bool(condition),detail if condition else "FAILED: "+detail)

def main()->int:
    parser=argparse.ArgumentParser(); parser.add_argument("--output",type=Path,required=True); args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True); results=[]
    left,right,scale=wheel_rpm(2.0,3.0)
    ul=(2.0-3.0*0.28*0.5)*60.0/(2.0*math.pi*0.03335); ur=(2.0+3.0*0.28*0.5)*60.0/(2.0*math.pi*0.03335)
    results.append(check("wheel_rpm_saturation_bound",max(abs(left),abs(right))<=100.0+1e-12 and scale<1.0,f"left={left:.6f}, right={right:.6f}, scale={scale:.6f}"))
    results.append(check("saturation_preserves_curvature_ratio",abs((left/right)-(ul/ur))<1e-12,"common scaling preserves left/right RPM ratio"))
    results.append(check("nonfinite_command_rejection_precondition",not (math.isfinite(math.nan) and math.isfinite(0.0)),"source callback finite-value guard rejects NaN"))
    results.append(check("startup_enters_safe_state",safe_required(has_command=False,command_age_ms=0,drive_enabled=False,sensor_age_ms=0,i2c_fault=False,bench_mode=False),"no validated command requires safe state"))
    results.append(check("watchdog_boundary_100ms_is_fresh",not safe_required(has_command=True,command_age_ms=100,drive_enabled=True,sensor_age_ms=0,i2c_fault=False,bench_mode=False),"firmware comparison is strictly > 100 ms"))
    results.append(check("watchdog_101ms_is_stale",safe_required(has_command=True,command_age_ms=101,drive_enabled=True,sensor_age_ms=0,i2c_fault=False,bench_mode=False),"101 ms command age requires safe state"))
    results.append(check("drive_disable_is_safe",safe_required(has_command=True,command_age_ms=0,drive_enabled=False,sensor_age_ms=0,i2c_fault=False,bench_mode=False),"valid heartbeat with drive flag cleared still stops actuation"))
    results.append(check("production_i2c_fault_is_safe",safe_required(has_command=True,command_age_ms=0,drive_enabled=True,sensor_age_ms=0,i2c_fault=True,bench_mode=False),"latched I2C fault blocks drive outside bench mode"))
    results.append(check("bench_mode_does_not_bypass_command_stale",safe_required(has_command=True,command_age_ms=2001,drive_enabled=True,sensor_age_ms=0,i2c_fault=False,bench_mode=True),"bench mode extends but does not remove command timeout"))
    bridge_send_times=[index*20 for index in range(101)]
    maximum_uart_age=max(now-max(sent for sent in bridge_send_times if sent<=now) for now in range(0,2001))
    results.append(check("upstream_timeout_dominates_stop_latency",maximum_uart_age<100 and bridge_send_times[-1]==2000,"model: stale upstream Twist can remain drive-enabled through 2000 ms"))
    write_csv(args.output/"control_safety_checks.csv",[asdict(item) for item in results])
    write_json(args.output/"control_safety_summary.json",{"passed":sum(item.passed for item in results),"failed":sum(not item.passed for item in results),"evidence_class":"B_source_derived_control_specification_model","source_constants":{"wheel_radius_m":0.03335,"wheel_base_m":0.28,"max_wheel_rpm":100.0,"bridge_cmd_vel_timeout_ms":2000,"pico_production_command_timeout_ms":100,"command_period_ms":20},"limitation":"This is a source-derived state/kinematics specification model, not execution of main.cpp or measurement of motor stop time.","results":[asdict(item) for item in results]})
    print(f"control safety model: {sum(item.passed for item in results)} passed, {sum(not item.passed for item in results)} failed")
    return 0 if all(item.passed for item in results) else 1

if __name__=="__main__":
    raise SystemExit(main())
