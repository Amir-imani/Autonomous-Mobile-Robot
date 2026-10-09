"""Compile and benchmark the production Neuro_PID C++ implementation on the host."""
from __future__ import annotations
import argparse,json,shutil,subprocess
from pathlib import Path
from common import write_csv,write_json
HERE=Path(__file__).resolve().parent; CODE=HERE.parent
def main():
    p=argparse.ArgumentParser(); p.add_argument("--output",type=Path,required=True); p.add_argument("--trials",type=int,default=3); p.add_argument("--iterations",type=int,default=1_000_000); a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=True)
    compiler=shutil.which("g++") or shutil.which("clang++")
    if not compiler: print("No C++ compiler found"); return 2
    exe=a.output/("neuro_pid_host_test.exe" if __import__("os").name=="nt" else "neuro_pid_host_test")
    cmd=[compiler,"-O3","-std=c++17",f"-I{HERE/'firmware_host/include'}",f"-I{CODE/'firmware/pico/include'}",str(HERE/'firmware_host/neuro_pid_host_test.cpp'),str(CODE/'firmware/pico/src/Neuro_PID.cpp'),"-o",str(exe)]
    r=subprocess.run(cmd,capture_output=True,text=True,check=False); (a.output/"neuro_pid_compile_stdout.txt").write_text(r.stdout); (a.output/"neuro_pid_compile_stderr.txt").write_text(r.stderr)
    if r.returncode:return r.returncode
    rows=[]
    for trial in range(1,a.trials+1):
        rr=subprocess.run([str(exe),str(a.iterations)],capture_output=True,text=True,check=False)
        if rr.returncode:return rr.returncode
        data=json.loads(rr.stdout); rows.append({"trial":trial,**data,"evidence_class":"A_host_execution_of_production_cpp"})
    write_csv(a.output/"neuro_pid_trials.csv",rows); write_json(a.output/"neuro_pid_status.json",{"compile_command":cmd,"trials":a.trials,"iterations":a.iterations})
    print(f"Neuro-PID host benchmark: {len(rows)} trials"); return 0
if __name__=="__main__": raise SystemExit(main())
