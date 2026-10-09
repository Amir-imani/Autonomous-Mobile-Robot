"""Create a compact final Markdown report for a completed experiment run."""
from __future__ import annotations
import argparse,csv,json
from pathlib import Path
def read_csv(p):
    with p.open(encoding="utf-8",newline="") as f:return list(csv.DictReader(f))
def main():
    p=argparse.ArgumentParser(); p.add_argument("--run-dir",type=Path,required=True); a=p.parse_args()
    manifest=json.loads((a.run_dir/"run_manifest.json").read_text(encoding="utf-8"))
    summary=read_csv(a.run_dir/"statistical_summary.csv") if (a.run_dir/"statistical_summary.csv").exists() else []
    lines=["# FINAL REPORT","",f"Run: {manifest.get('run_id','unknown')}",f"Status: **{manifest.get('status','unknown')}**","","## Evidence policy","",manifest.get("evidence_policy",""),"","## Aggregate metrics","", "| Study | Scenario | Metric | Mean | P95 | Evidence |","|---|---|---|---:|---:|---|"]
    for r in summary: lines.append(f"| {r.get('study','')} | {r.get('scenario','')} | {r.get('metric','')} | {r.get('mean','')} | {r.get('p95','')} | {r.get('evidence_class','')} |")
    lines += ["","## Interpretation","","Host/synthetic results characterize the audited software and sensitivity of the architecture. They are not measurements of final Raspberry Pi 5, GPU, physical-network, SLAM, localization, or robot-control performance.",""]
    (a.run_dir/"FINAL_REPORT.md").write_text("\n".join(lines),encoding="utf-8"); print("final report generated"); return 0
if __name__=="__main__":raise SystemExit(main())
