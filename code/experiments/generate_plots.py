"""Generate dependency-free SVG figures from benchmark CSV files."""
from __future__ import annotations
import argparse,csv,html
from pathlib import Path
def read(path):
    with path.open(encoding="utf-8",newline="") as f:return list(csv.DictReader(f))
def bar_svg(title,labels,values,ylabel):
    w,h=900,520; ml,mr,mt,mb=90,30,70,110; pw=w-ml-mr; ph=h-mt-mb; vmax=max(values+[1e-9])*1.15; out=[]
    out.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">')
    out.append(f'<text x="{w/2}" y="35" text-anchor="middle" font-size="22">{html.escape(title)}</text>')
    out.append(f'<text x="20" y="{h/2}" transform="rotate(-90 20 {h/2})" text-anchor="middle">{html.escape(ylabel)}</text>')
    bw=pw/max(1,len(values))*0.62
    for i,(lab,val) in enumerate(zip(labels,values)):
        x=ml+(i+0.5)*pw/len(values); bh=ph*val/vmax; y=mt+ph-bh
        out.append(f'<rect x="{x-bw/2:.1f}" y="{y:.1f}" width="{bw:.1f}" height="{bh:.1f}" fill="#4c78a8"/>')
        out.append(f'<text x="{x:.1f}" y="{mt+ph+25}" text-anchor="middle" font-size="12">{html.escape(lab)}</text>')
        out.append(f'<text x="{x:.1f}" y="{y-6:.1f}" text-anchor="middle" font-size="12">{val:.3f}</text>')
    out.append(f'<line x1="{ml}" y1="{mt+ph}" x2="{ml+pw}" y2="{mt+ph}" stroke="black"/>'); out.append('</svg>'); return "\n".join(out)
def main():
    p=argparse.ArgumentParser(); p.add_argument("--run-dir",type=Path,required=True); a=p.parse_args(); fig=a.run_dir/"figures"; fig.mkdir(exist_ok=True)
    off=read(a.run_dir/"offload_trials.csv"); selected=[r for r in off if r["scenario"]=="distributed_loopback_process" and r["trial"]=="1" and r["condition"] in ("baseline","delay_50ms","delay_100ms")]
    (fig/"latency_vs_delay.svg").write_text(bar_svg("Distributed offload latency",[r["condition"] for r in selected],[float(r["latency_mean_ms"]) for r in selected],"Mean latency (ms)"),encoding="utf-8")
    stress=read(a.run_dir/"stress_trials.csv"); first={}
    for r in stress:first.setdefault(r["scenario"],r)
    (fig/"control_timing_under_load.svg").write_text(bar_svg("50 Hz scheduling lateness",list(first),[float(r["scheduling_lateness_p99_ms"]) for r in first.values()],"P99 lateness (ms)"),encoding="utf-8")
    print(f"plots: {len(list(fig.glob('*.svg')))} SVG files"); return 0
if __name__=="__main__":raise SystemExit(main())
