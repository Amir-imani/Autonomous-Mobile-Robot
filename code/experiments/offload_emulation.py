"""Application-layer offload emulation for architecture sensitivity analysis."""
from __future__ import annotations
import argparse,hashlib,math,multiprocessing as mp,os,random,socket,struct,time
from pathlib import Path
from common import describe,environment_metadata,json_safe,write_csv,write_json
from resource_probe import read_process

def workload(payload:bytes,rounds:int)->bytes:
    digest=payload
    for _ in range(rounds):digest=hashlib.sha256(digest).digest()
    return digest

def recvall(sock,n):
    data=b""
    while len(data)<n:
        part=sock.recv(n-len(data))
        if not part: raise EOFError
        data+=part
    return data

def worker(port_pipe,rounds):
    server=socket.socket(); server.bind(("127.0.0.1",0)); server.listen(1); port_pipe.send((server.getsockname()[1],os.getpid()))
    conn,_=server.accept()
    try:
        while True:
            header=conn.recv(4)
            if not header:break
            size=struct.unpack("!I",header)[0]; payload=recvall(conn,size); result=workload(payload,rounds); conn.sendall(struct.pack("!I",len(result))+result)
    finally: conn.close(); server.close()

def run_condition(name,delay_ms,jitter_ms,loss,payload,rounds,frames,rng,distributed):
    rows=[]; proc=None; sock=None; child_pid=None
    if distributed:
        parent,child=mp.Pipe(duplex=False); proc=mp.Process(target=worker,args=(child,rounds)); proc.start(); port,child_pid=parent.recv(); sock=socket.create_connection(("127.0.0.1",port))
    for index in range(frames):
        if rng.random()<loss:
            rows.append({"frame":index,"delivered":0,"end_to_end_ms":"","processing_ms":"","scenario":"distributed_loopback_process" if distributed else "monolithic_in_process","condition":name}); continue
        extra=max(0.0,delay_ms+rng.uniform(-jitter_ms,jitter_ms))
        t0=time.perf_counter(); time.sleep(extra/1000.0); p0=time.perf_counter()
        if distributed:
            sock.sendall(struct.pack("!I",len(payload))+payload); n=struct.unpack("!I",recvall(sock,4))[0]; recvall(sock,n)
        else:workload(payload,rounds)
        p1=time.perf_counter()
        rows.append({"frame":index,"delivered":1,"end_to_end_ms":(p1-t0)*1000.0,"processing_ms":(p1-p0)*1000.0,"scenario":"distributed_loopback_process" if distributed else "monolithic_in_process","condition":name})
    if sock:sock.close()
    if proc:proc.join(timeout=2); proc.terminate() if proc.is_alive() else None
    return rows,child_pid

def main():
    p=argparse.ArgumentParser(); p.add_argument("--output",type=Path,required=True); p.add_argument("--trials",type=int,default=3); p.add_argument("--frames",type=int,default=60); p.add_argument("--payload-kib",type=int,default=150); p.add_argument("--rounds",type=int,default=64); a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=True)
    payload=bytes((i*17+31)&0xff for i in range(a.payload_kib*1024)); allrows=[]; trials=[]
    conditions=[("baseline",0,0,0.0),("delay_50ms",50,0,0.0),("delay_100ms",100,0,0.0),("jitter_25ms",25,10,0.0),("loss_1pct",0,0,0.01)]
    for distributed in (False,True):
        scenario="distributed_loopback_process" if distributed else "monolithic_in_process"
        for cond,delay,jitter,loss in conditions:
            for trial in range(1,a.trials+1):
                rng=random.Random(20260905+trial+delay+(1 if distributed else 0)); rows,_=run_condition(cond,delay,jitter,loss,payload,a.rounds,a.frames,rng,distributed)
                for r in rows:r.update({"trial":trial,"payload_kib":a.payload_kib,"evidence_class":"B_application_layer_network_emulation" if distributed else "B_host_process_emulation"})
                allrows.extend(rows); delivered=[r for r in rows if r["delivered"]==1]; lat=[float(r["end_to_end_ms"]) for r in delivered]
                jitter_values=[abs(lat[i]-lat[i-1]) for i in range(1,len(lat))]
                duration=sum(float(r["end_to_end_ms"] or 0) for r in delivered)/1000.0
                trials.append({"scenario":scenario,"condition":cond,"trial":trial,"sent_frames":a.frames,"delivered_frames":len(delivered),"drop_count":a.frames-len(delivered),"latency_mean_ms":describe(lat)["mean"],"latency_p95_ms":describe(lat)["p95"],"jitter_mean_abs_consecutive_latency_ms":describe(jitter_values)["mean"],"achieved_fps":len(delivered)/duration if duration>0 else math.nan,"evidence_class":"B_application_layer_network_emulation" if distributed else "B_host_process_emulation"})
    write_csv(a.output/"offload_frames.csv",allrows); write_csv(a.output/"offload_trials.csv",trials)
    write_json(a.output/"offload_method.json",json_safe({"environment":environment_metadata(),"payload_kib":a.payload_kib,"surrogate":"iterated SHA-256","rounds":a.rounds,"limitations":["Loopback TCP is not DDS or a physical network.","SHA-256 is a controlled workload, not neural-network inference."]}))
    print(f"offload emulation: {len(trials)} trial-condition summaries"); return 0
if __name__=="__main__":
    mp.freeze_support(); raise SystemExit(main())
