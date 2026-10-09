"""Aggregate one benchmark run and write thesis-safe tables and interpretation."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable

from common import describe, json_safe, write_csv, write_json


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def finite(rows: Iterable[dict[str, str]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        raw = row.get(key, "")
        try:
            value = float(raw)
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return values


def metric_row(study: str, scenario: str, metric: str, unit: str, values: Iterable[float], evidence_class: str, poster_eligible: str, notes: str) -> dict[str, object]:
    return {"study": study,"scenario": scenario,"metric": metric,"unit": unit,**describe(values),"evidence_class": evidence_class,"poster_eligible": poster_eligible,"notes": notes}


def aggregate(run_dir: Path) -> list[dict[str, object]]:
    summary: list[dict[str, object]] = []
    core = read_csv(run_dir / "core_microbenchmarks.csv")
    for benchmark in sorted({row["benchmark"] for row in core}):
        selected = [row for row in core if row["benchmark"] == benchmark]
        summary.append(metric_row("production_core_microbenchmark", benchmark, "execution_time", "ns/operation", finite(selected, "ns_per_operation"), "A_direct_software_execution", "conditional", "Development-laptop Python timing; not Raspberry Pi 5 timing."))
        summary.append(metric_row("production_core_microbenchmark", benchmark, "throughput", "operations/s", finite(selected, "throughput_hz"), "A_direct_software_execution", "conditional", "Development-laptop Python timing; production pure class loaded unchanged."))

    neuro_path = run_dir / "neuro_pid_trials.csv"
    if neuro_path.exists():
        neuro = read_csv(neuro_path)
        summary.append(metric_row("production_neuro_pid_cpp", "host_optimized_cpp", "forward_pass_time", "ns/inference", finite(neuro, "ns_per_inference"), "A_host_execution_of_production_cpp", "conditional", "Exact deployed C++ math on laptop CPU; not RP2040 timing/control accuracy."))

    frames = read_csv(run_dir / "offload_frames.csv")
    trials = read_csv(run_dir / "offload_trials.csv")
    keys = sorted({(row["scenario"], row["condition"]) for row in frames})
    for scenario, condition in keys:
        selected_frames = [row for row in frames if row["scenario"] == scenario and row["condition"] == condition and row.get("delivered") == "1"]
        selected_trials = [row for row in trials if row["scenario"] == scenario and row["condition"] == condition]
        evidence = "B_host_process_emulation" if scenario == "monolithic_in_process" else "B_application_layer_network_emulation"
        summary.append(metric_row("offload_emulation", f"{scenario}:{condition}", "end_to_end_latency", "ms", finite(selected_frames, "end_to_end_ms"), evidence, "emulation_only", "Loopback/process timing with SHA-256 surrogate; not DDS or AI inference."))
        summary.append(metric_row("offload_emulation", f"{scenario}:{condition}", "latency_jitter", "ms mean absolute consecutive difference", finite(selected_trials, "jitter_mean_abs_consecutive_latency_ms"), evidence, "emulation_only", "Exact jitter definition is mean |L[n]-L[n-1]| per trial."))
        summary.append(metric_row("offload_emulation", f"{scenario}:{condition}", "achieved_throughput", "frames/s", finite(selected_trials, "achieved_fps"), evidence, "emulation_only", "Synchronous request/result throughput, not camera FPS."))

    summary.append(metric_row("analytical_capacity", "pico_uart_nominal_50hz", "wire_utilization", "percent of 921600 baud", [100.0 * 50_000.0 / 921_600.0], "C_analytical_estimate", "yes_if_labeled", "80-byte telemetry + 20-byte command, 10 serial bits/byte, both at 50 Hz."))
    payload_kib = float(frames[0]["payload_kib"])
    summary.append(metric_row("analytical_capacity", f"synthetic_{payload_kib:g}KiB_at_30fps", "image_payload_bandwidth", "Mbit/s", [payload_kib * 1024.0 * 30.0 * 8.0 / 1_000_000.0], "C_analytical_estimate", "yes_if_labeled", "Payload only; excludes DDS/RTPS/IP/link overhead and is not measured camera JPEG size."))

    stress_path = run_dir / "stress_ticks.csv"
    if stress_path.exists():
        stress = read_csv(stress_path)
        for scenario in sorted({row["scenario"] for row in stress}):
            selected = [row for row in stress if row["scenario"] == scenario]
            summary.append(metric_row("concurrent_load_emulation", scenario, "scheduling_lateness", "ms", finite(selected, "scheduling_lateness_ms"), "B_host_cpu_stress_emulation", "emulation_only", "Absolute 50 Hz deadline lateness; Windows process load, not ROS executor."))
            summary.append(metric_row("concurrent_load_emulation", scenario, "interarrival_jitter", "ms absolute period error", finite(selected, "interarrival_jitter_ms"), "B_host_cpu_stress_emulation", "emulation_only", "Absolute difference between actual inter-start interval and 20 ms."))
            summary.append(metric_row("concurrent_load_emulation", scenario, "callback_duration", "ms", finite(selected, "callback_duration_ms"), "B_host_cpu_stress_emulation", "emulation_only", "Production kinematics call only; excludes rclpy/executor/DDS."))
    return summary


def select(summary: list[dict[str, object]], scenario_fragment: str, metric: str) -> dict[str, object]:
    for row in summary:
        if scenario_fragment in str(row["scenario"]) and row["metric"] == metric:
            return row
    raise KeyError((scenario_fragment, metric))


def write_comparison(run_dir: Path, summary: list[dict[str, object]]) -> None:
    local = select(summary, "monolithic_in_process:baseline", "end_to_end_latency")
    distributed = select(summary, "distributed_loopback_process:baseline", "end_to_end_latency")
    delay_100 = select(summary, "delay_100ms", "end_to_end_latency")
    lines = ["# Comparison table","","| Scenario | Evidence | Mean latency (ms) | P95 (ms) | Interpretation |","|---|---|---:|---:|---|",f"| Monolithic in-process | B: host emulation | {local['mean']:.3f} | {local['p95']:.3f} | Same-process SHA-256 surrogate |",f"| Distributed loopback baseline | B: host emulation | {distributed['mean']:.3f} | {distributed['p95']:.3f} | Adds TCP, copy, IPC, and scheduling |",f"| Distributed +100 ms input delay | B: application emulation | {delay_100['mean']:.3f} | {delay_100['p95']:.3f} | Controlled delay; not tc/DDS |","","This table is not evidence of Pi 5, ROS 2 DDS, GPU inference, or physical robot performance."]
    (run_dir / "comparison_table.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_persian_reports(run_dir: Path, summary: list[dict[str, object]]) -> None:
    local = select(summary, "monolithic_in_process:baseline", "end_to_end_latency")
    distributed = select(summary, "distributed_loopback_process:baseline", "end_to_end_latency")
    delay_50 = select(summary, "delay_50ms", "end_to_end_latency")
    serial = select(summary, "pico_uart_nominal_50hz", "wire_utilization")
    stress_16 = select(summary, "background_workers_16", "scheduling_lateness")
    checks = json.loads((run_dir / "core_summary.json").read_text(encoding="utf-8"))["checks"]
    thesis = f"""# جمع‌بندی فنی آماده برای پایان‌نامه

## دامنه اعتبار

این آزمایش‌ها روی رایانه توسعه و بدون ربات، Raspberry Pi 5، ROS 2/DDS، دوربین و مدل YOLO اجرا شده‌اند. بنابراین نتایج زمان‌سنجی شبکه در رده «شبیه‌سازی فرایندی/شبکه‌ای» و آزمون‌های پروتکل، اودومتری و LiDAR در رده «اجرای مستقیم منطق نرم‌افزار» هستند. هیچ عددی به‌عنوان عملکرد فیزیکی ربات یا GPU گزارش نمی‌شود.

## یافته‌ها

- {checks['behavior_verifications_passed']} آزمون رفتاری منطق مستقل از سخت‌افزار گذشت، {checks['failed']} آزمون شکست خورد و {checks['detected_defects']} ممیزیِ جداگانه یک عیب پیکربندی را با موفقیت آشکار کرد. این نتیجه صحت مسیرهای نرم‌افزاری آزموده‌شده را نشان می‌دهد، نه صحت حسگر و عملگر واقعی.
- میانگین تأخیر سناریوی تک‌فرایندی {local['mean']:.3f} ms و سناریوی دوفرایندی روی loopback برابر {distributed['mean']:.3f} ms بود. این اختلاف تنها هزینه جداسازی فرایند/TCP روی همین میزبان و بار محاسباتی جایگزین SHA-256 را بیان می‌کند.
- با اعمال تأخیر مصنوعی 50 ms در لایه کاربرد، میانگین انتهابه‌انتها {delay_50['mean']:.3f} ms شد. چون DDS و شبکه فیزیکی در مسیر نبودند، این نتیجه فقط حساسیت معماری را نشان می‌دهد.
- بار نظری لینک سریال دوطرفه در نرخ 50 Hz حدود {serial['mean']:.3f}% ظرفیت 921600 baud است؛ این مقدار از اندازه ثابت فریم‌ها محاسبه شده و اندازه‌گیری ترافیک واقعی نیست.
- زیر 16 worker محاسباتی مصنوعی، P99 دیرکرد شروع حلقه 50 Hz برابر {stress_16['p99']:.3f} ms بود. این تنها رفتار scheduler ویندوز در فشار مصنوعی است و معادل executor در ROS 2 نیست.
- ممیزی واحدها نشان داد firmware مقدار `lidar_distance_mm` را می‌فرستد، ولی launch ضریب 0.01 m/unit دارد؛ در صورت تأیید پروتکل حسگر، مقدار سازگار 0.001 است.

## پاسخ به فرضیه معماری

از داده فعلی نمی‌توان ثابت کرد offload بار CPU Raspberry Pi 5 را کاهش می‌دهد، چون Pi و گره GPU واقعی اجرا نشده‌اند. آزمایش حاضر فقط نشان می‌دهد جداسازی محاسبه هزینه ارتباطی و حساسیت تقریباً مستقیم به تأخیر افزوده دارد. کنترل حرکت، watchdog، serial bridge و حداقل obstacle stop باید محلی بمانند. تشخیص معنایی می‌تواند دورکار باشد؛ Visual Odometry برای حلقه کنترل باید fallback محلی و سقف سن فریم داشته باشد.

## نتیجه قابل دفاع

قابل دفاع است که هسته نرم‌افزاری منتخب تست‌پذیر است، framing/CRC در ورودی‌های مصنوعی بازیابی می‌شود، مدل کینماتیکی با حل تحلیلی سازگار است، و معماری offload به بودجه تأخیر شبکه حساس است. ادعای کاهش مصرف Pi، FPS واقعی YOLO، دقت SLAM، خطای مکان‌یابی یا پایداری کنترل بدون سخت‌افزار/ROS 2 معتبر نیست.
"""
    (run_dir / "THESIS_INTERPRETATION_FA.md").write_text(thesis, encoding="utf-8")

    poster = f"""# POSTER-READY RESULTS

1. **آزمون هسته نرم‌افزار:** {checks['behavior_verifications_passed']} آزمون رفتاری پروتکل، اودومتری و LiDAR بدون شکست اجرا شد و یک ممیزی مستقل ناسازگاری واحد را یافت. معیار: pass/fail. نمودار پیشنهادی: `core_throughput.svg`. هشدار: بدون سخت‌افزار.
2. **هزینه جداسازی فرایند:** میانگین تأخیر از {local['mean']:.3f} ms در حالت تک‌فرایندی به {distributed['mean']:.3f} ms در loopback رسید. معیار: mean و P95. نمودار: `local_vs_distributed_latency.svg`. هشدار: TCP loopback و SHA-256، نه DDS/YOLO.
3. **حساسیت به شبکه:** تأخیر مصنوعی 50 ms میانگین انتهابه‌انتها را به {delay_50['mean']:.3f} ms رساند. معیار: latency vs added delay. نمودار: `latency_vs_delay.svg`. هشدار: emulation لایه کاربرد.
4. **فشار هم‌زمان:** زیر 16 worker مصنوعی، P99 دیرکرد حلقه 50 Hz برابر {stress_16['p99']:.3f} ms بود. معیار: deadline lateness. نمودار: `control_timing_under_load.svg`. هشدار: scheduler ویندوز، نه ROS executor.
5. **یافته ایمنی داده:** ناسازگاری 10 برابری محتمل در مقیاس LiDAR از روی کد شناسایی شد؛ بار نظری سریال نیز فقط {serial['mean']:.3f}% محاسبه شد. هشدار: اولی نیازمند تأیید با فاصله مرجع و دومی برآورد تحلیلی است.

بهترین نمودارها برای پوستر: `latency_vs_delay.svg`، `resource_split.svg` و `control_timing_under_load.svg`. همه باید برچسب «شبیه‌سازی روی میزبان توسعه» داشته باشند.
"""
    (run_dir / "POSTER_READY_FA.md").write_text(poster, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    args = parser.parse_args()
    summary = aggregate(args.run_dir)
    write_csv(args.run_dir / "statistical_summary.csv", summary)
    write_json(args.run_dir / "statistical_summary.json", json_safe(summary))
    write_comparison(args.run_dir, summary)
    write_persian_reports(args.run_dir, summary)
    print(f"analysis: {len(summary)} aggregate metric rows")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
