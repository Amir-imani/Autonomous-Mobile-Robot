"""Run the complete hardware-free campaign and preserve commands/logs."""
from __future__ import annotations
import argparse
from pathlib import Path
import subprocess,sys,time
from common import environment_metadata,write_json
HERE=Path(__file__).resolve().parent

def main()->int:
    p=argparse.ArgumentParser()
    p.add_argument("--output",type=Path); p.add_argument("--trials",type=int,default=3); p.add_argument("--frames",type=int,default=60)
    p.add_argument("--core-iterations",type=int,default=5000); p.add_argument("--neuro-iterations",type=int,default=1_000_000)
    p.add_argument("--payload-kib",type=int,default=150); p.add_argument("--surrogate-rounds",type=int,default=64); p.add_argument("--stress-duration",type=float,default=5.0)
    args=p.parse_args()
    run_id=time.strftime("run_%Y%m%dT%H%M%SZ",time.gmtime()); run_dir=(args.output or (HERE/"results"/run_id)).resolve(); run_dir.mkdir(parents=True,exist_ok=False)
    commands=[
        [sys.executable,str(HERE/"codebase_audit.py"),"--output",str(run_dir)],
        [sys.executable,str(HERE/"core_logic_benchmark.py"),"--output",str(run_dir),"--trials",str(args.trials),"--iterations",str(args.core_iterations)],
        [sys.executable,str(HERE/"host_neuro_benchmark.py"),"--output",str(run_dir),"--trials",str(args.trials),"--iterations",str(args.neuro_iterations)],
        [sys.executable,str(HERE/"offload_emulation.py"),"--output",str(run_dir),"--trials",str(args.trials),"--frames",str(args.frames),"--payload-kib",str(args.payload_kib),"--rounds",str(args.surrogate_rounds)],
        [sys.executable,str(HERE/"control_safety_checks.py"),"--output",str(run_dir)],
        [sys.executable,str(HERE/"stress_benchmark.py"),"--output",str(run_dir),"--trials",str(args.trials),"--duration",str(args.stress_duration),"--workers","0,1,4,16"],
        [sys.executable,str(HERE/"analyze_results.py"),"--run-dir",str(run_dir)],
        [sys.executable,str(HERE/"generate_plots.py"),"--run-dir",str(run_dir)],
    ]
    records=[]; failed=False
    for index,command in enumerate(commands,start=1):
        started=time.perf_counter(); result=subprocess.run(command,capture_output=True,text=True,check=False)
        record={"step":index,"command":command,"returncode":result.returncode,"elapsed_s":time.perf_counter()-started,"stdout_file":f"step_{index:02d}_stdout.txt","stderr_file":f"step_{index:02d}_stderr.txt"}
        (run_dir/record["stdout_file"]).write_text(result.stdout,encoding="utf-8"); (run_dir/record["stderr_file"]).write_text(result.stderr,encoding="utf-8")
        records.append(record); print(result.stdout.strip() or f"step {index} completed",flush=True)
        if result.returncode!=0:
            failed=True; print(f"step {index} failed; see {record['stderr_file']}",file=sys.stderr,flush=True)
            if index<=6: break
    manifest={"run_id":run_id,"status":"failed" if failed else "complete","environment":environment_metadata(),"parameters":vars(args)|{"output":str(run_dir)},"steps":records,"evidence_policy":"Every artifact distinguishes direct software execution, simulation/emulation, analytical estimate, and hardware-deferred work."}
    write_json(run_dir/"run_manifest.json",manifest)
    if not failed:
        cmd=[sys.executable,str(HERE/"generate_final_report.py"),"--run-dir",str(run_dir)]
        started=time.perf_counter(); rr=subprocess.run(cmd,capture_output=True,text=True,check=False)
        (run_dir/"final_report_stdout.txt").write_text(rr.stdout,encoding="utf-8"); (run_dir/"final_report_stderr.txt").write_text(rr.stderr,encoding="utf-8")
        manifest["report_generation"]={"command":cmd,"returncode":rr.returncode,"elapsed_s":time.perf_counter()-started}
        if rr.returncode!=0: failed=True; manifest["status"]="failed"
        write_json(run_dir/"run_manifest.json",manifest)
    (HERE/"results"/"LATEST.txt").parent.mkdir(parents=True,exist_ok=True); (HERE/"results"/"LATEST.txt").write_text(str(run_dir)+"\n",encoding="utf-8")
    print(f"run directory: {run_dir}"); return 1 if failed else 0
if __name__=="__main__": raise SystemExit(main())
