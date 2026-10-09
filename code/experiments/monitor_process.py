"""Monitor CPU/RSS for an external process during reproducibility runs."""
from __future__ import annotations
import argparse,csv,time
from pathlib import Path
from resource_probe import read_process
def main():
    p=argparse.ArgumentParser(); p.add_argument("pid",type=int); p.add_argument("--output",type=Path,required=True); p.add_argument("--interval",type=float,default=0.1); p.add_argument("--duration",type=float,default=10.0); a=p.parse_args()
    rows=[]; start=time.perf_counter()
    while time.perf_counter()-start<a.duration:
        try:r=read_process(a.pid)
        except (OSError,ProcessLookupError):break
        rows.append({"elapsed_s":time.perf_counter()-start,"cpu_seconds":r.cpu_seconds,"rss_mb":r.rss_bytes/1048576.0}); time.sleep(a.interval)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=["elapsed_s","cpu_seconds","rss_mb"]); w.writeheader(); w.writerows(rows)
    return 0
if __name__=="__main__":raise SystemExit(main())
