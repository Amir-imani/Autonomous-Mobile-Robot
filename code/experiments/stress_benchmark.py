"""Measure a synthetic 50 Hz callback schedule under concurrent CPU workloads."""
from __future__ import annotations
import argparse,hashlib,multiprocessing as mp,os,time
from pathlib import Path
from common import describe,environment_metadata,json_safe,write_csv,write_json
from resource_probe import read_process
from source_harness import load_odometry_symbols

def cpu_worker(stop,ready,payload_bytes):
    payload=bytes((i*31+17)&0xff for i in range(payload_bytes)); digest=b""; ready.send(os.getpid())
    while not stop.is_set(): digest=hashlib.sha256(payload+digest).digest()

def resource_delta(before,after):
    wall=max(after.monotonic_s-before.monotonic_s,1e-12); return 100.0*(after.cpu_seconds-before.cpu_seconds)/wall,max(before.rss_bytes,after.rss_bytes)/1048576.0

def one_trial(worker_count,duration_s,trial):
    stop=mp.Event(); processes=[]; pids=[]
    for _ in range(worker_count):
        parent,child=mp.Pipe(duplex=False); process=mp.Process(target=cpu_worker,args=(stop,child,256*1024),daemon=True); process.start(); pids.append(parent.recv()); processes.append(process)
    wb=[read_process(pid) for pid in pids]; pb=read_process(os.getpid())
    K=load_odometry_symbols()["DifferentialDriveKinematics"]; model=K(0.03335,0.28); period=0.02; ticks=round(duration_s/period); deadline=time.perf_counter(); previous=None; rows=[]
    for tick in range(ticks):
        remaining=deadline-time.perf_counter()
        if remaining>0: time.sleep(remaining)
        actual=time.perf_counter(); late=max(0.0,actual-deadline)*1000.0; c0=time.perf_counter_ns(); model.update(30.0+(tick%3),30.0-(tick%3),period); callback=(time.perf_counter_ns()-c0)/1e6
        inter="" if previous is None else abs((actual-previous)-period)*1000.0
        rows.append({"scenario":f"background_workers_{worker_count}","trial":trial,"tick":tick,"scheduled_period_ms":20.0,"scheduling_lateness_ms":late,"interarrival_jitter_ms":inter,"callback_duration_ms":callback,"missed_period":int(late>20.0),"evidence_class":"B_host_cpu_stress_emulation"})
        previous=actual; deadline+=period
    pa=read_process(os.getpid()); wa=[read_process(pid) for pid in pids]; stop.set()
    for process in processes:
        process.join(timeout=3)
        if process.is_alive(): process.terminate(); process.join(timeout=1)
    pc,pr=resource_delta(pb,pa); wr=[resource_delta(b,a) for b,a in zip(wb,wa)]
    latevals=[float(r["scheduling_lateness_ms"]) for r in rows]; jit=[float(r["interarrival_jitter_ms"]) for r in rows if r["interarrival_jitter_ms"]!=""]; cb=[float(r["callback_duration_ms"]) for r in rows]
    summary={"scenario":f"background_workers_{worker_count}","trial":trial,"ticks":len(rows),"missed_periods":sum(int(r["missed_period"]) for r in rows),"scheduling_lateness_mean_ms":describe(latevals)["mean"],"scheduling_lateness_p95_ms":describe(latevals)["p95"],"scheduling_lateness_p99_ms":describe(latevals)["p99"],"interarrival_jitter_mean_ms":describe(jit)["mean"],"callback_duration_mean_ms":describe(cb)["mean"],"callback_duration_p99_ms":describe(cb)["p99"],"parent_cpu_percent_one_core":pc,"parent_rss_mb":pr,"workers_cpu_percent_one_core_sum":sum(x[0] for x in wr),"workers_rss_mb_sum":sum(x[1] for x in wr),"evidence_class":"B_host_cpu_stress_emulation"}
    return rows,summary

def main():
    p=argparse.ArgumentParser(); p.add_argument("--output",type=Path,required=True); p.add_argument("--trials",type=int,default=3); p.add_argument("--duration",type=float,default=5.0); p.add_argument("--workers",default="0,1,4,16"); a=p.parse_args(); counts=[int(v) for v in a.workers.split(",")]; a.output.mkdir(parents=True,exist_ok=True)
    allrows=[]; summaries=[]
    for count in counts:
        for trial in range(1,a.trials+1):
            rows,summary=one_trial(count,a.duration,trial); allrows.extend(rows); summaries.append(summary)
    write_csv(a.output/"stress_ticks.csv",allrows); write_csv(a.output/"stress_trials.csv",summaries)
    write_json(a.output/"stress_method.json",json_safe({"environment":environment_metadata(),"trials":a.trials,"duration_s_per_trial":a.duration,"worker_counts":counts,"target_period_ms":20.0,"workload":"independent 256 KiB SHA-256 loops; production kinematics callback","evidence_class":"B_host_cpu_stress_emulation"}))
    print(f"stress emulation: {len(summaries)} trial-condition summaries"); return 0
if __name__=="__main__": mp.freeze_support(); raise SystemExit(main())
