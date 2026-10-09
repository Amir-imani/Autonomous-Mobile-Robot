"""Static inventory for the reorganized thesis repository."""
from __future__ import annotations
import argparse,ast,shutil,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
from common import environment_metadata,write_csv,write_json
from source_harness import ROOT
EXCLUDED_PARTS={".git",".pio","__pycache__","results","build","install","log"}

def visible_files():
    return sorted(p for p in ROOT.rglob("*") if p.is_file() and not any(part in EXCLUDED_PARTS for part in p.parts))
def package_inventory(files):
    out=[]
    for m in files:
        if m.name!="package.xml": continue
        root=ET.parse(m).getroot(); export=root.find("export"); bt="unknown"
        if export is not None and export.find("build_type") is not None: bt=(export.findtext("build_type") or "unknown").strip()
        out.append({"package":(root.findtext("name") or "unknown").strip(),"build_type":bt,"path":str(m.parent.relative_to(ROOT))})
    return out
def python_syntax(files):
    rows=[]
    for p in files:
        if p.suffix!=".py": continue
        try: ast.parse(p.read_text(encoding="utf-8"),filename=str(p)); rows.append({"file":str(p.relative_to(ROOT)),"status":"passed","error":""})
        except (SyntaxError,UnicodeError) as exc: rows.append({"file":str(p.relative_to(ROOT)),"status":"failed","error":str(exc)})
    return rows
def command_status(command,args):
    exe=shutil.which(command)
    if not exe:return {"tool":command,"available":False,"path":"","version":""}
    try:
        r=subprocess.run([exe,*args],capture_output=True,text=True,timeout=10,check=False); lines=(r.stdout or r.stderr).splitlines()
        return {"tool":command,"available":True,"path":exe,"version":lines[0] if lines else f"exit={r.returncode}"}
    except Exception as exc:return {"tool":command,"available":True,"path":exe,"version":f"probe failed: {exc}"}
def main():
    p=argparse.ArgumentParser(); p.add_argument("--output",type=Path,required=True); a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=True)
    files=visible_files(); packages=package_inventory(files); syntax=python_syntax(files)
    tools=[command_status("python",["--version"]),command_status("ros2",["--help"]),command_status("colcon",["--help"]),command_status("pio",["--version"]),command_status("g++",["--version"]),command_status("cmake",["--version"]),command_status("nvidia-smi",["--query-gpu=name","--format=csv,noheader"])]
    write_csv(a.output/"packages.csv",packages); write_csv(a.output/"python_syntax.csv",syntax); write_csv(a.output/"tool_availability.csv",tools)
    artifacts={"rosbags":[str(x.relative_to(ROOT)) for x in files if x.suffix.lower() in {".bag",".db3",".mcap"}],"vision_models":[str(x.relative_to(ROOT)) for x in files if x.suffix.lower() in {".onnx",".engine",".pt",".pth",".weights"}],"videos":[str(x.relative_to(ROOT)) for x in files if x.suffix.lower() in {".mp4",".avi",".mov",".mkv",".webm"}]}
    write_json(a.output/"codebase_inventory.json",{"environment":environment_metadata(),"scope":str(ROOT),"visible_file_count":len(files),"ros_package_count":len(packages),"packages":packages,"artifacts":artifacts,"python_syntax":{"checked":len(syntax),"passed":sum(r["status"]=="passed" for r in syntax),"failed":sum(r["status"]=="failed" for r in syntax)},"tools":tools,"negative_findings":["No Nav2 package/configuration in thesis snapshot.","No final SLAM package/configuration in repository snapshot.","No AI model/video/dataset bundled in source repository."]})
    failed=sum(r["status"]=="failed" for r in syntax); print(f"audit: {len(packages)} ROS packages, {len(syntax)} Python files, {failed} syntax failures"); return 1 if failed else 0
if __name__=="__main__": raise SystemExit(main())
