"""Dependency-free helpers shared by the hardware-free benchmark suite."""

from __future__ import annotations

import csv
import json
import math
import os
import platform
import socket
import statistics
import sys
import time
from pathlib import Path
from typing import Iterable, Mapping, Sequence

SCHEMA_VERSION = "1.0"

def percentile(values: Sequence[float], quantile: float) -> float:
    if not values:
        return math.nan
    ordered=sorted(float(value) for value in values)
    if len(ordered)==1:
        return ordered[0]
    position=(len(ordered)-1)*quantile
    lower=math.floor(position); upper=math.ceil(position)
    if lower==upper:
        return ordered[lower]
    fraction=position-lower
    return ordered[lower]*(1.0-fraction)+ordered[upper]*fraction

def describe(values: Iterable[float]) -> dict[str,float|int]:
    finite=[float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {"count":0,"mean":math.nan,"median":math.nan,"min":math.nan,"max":math.nan,"stddev":math.nan,"p95":math.nan,"p99":math.nan}
    return {"count":len(finite),"mean":statistics.fmean(finite),"median":statistics.median(finite),"min":min(finite),"max":max(finite),"stddev":statistics.stdev(finite) if len(finite)>1 else 0.0,"p95":percentile(finite,0.95),"p99":percentile(finite,0.99)}

def environment_metadata() -> dict[str,object]:
    return {"schema_version":SCHEMA_VERSION,"captured_utc":time.strftime("%Y-%m-%dT%H:%M:%SZ",time.gmtime()),"hostname":socket.gethostname(),"platform":platform.platform(),"python":sys.version.replace("\n"," "),"logical_cpu_count":os.cpu_count(),"timer":"time.perf_counter_ns (monotonic high-resolution)"}

def write_json(path: Path,payload: object) -> None:
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(payload,indent=2,sort_keys=True,allow_nan=False),encoding="utf-8")
    temporary.replace(path)

def json_safe(value: object) -> object:
    if isinstance(value,float) and not math.isfinite(value):
        return None
    if isinstance(value,dict):
        return {str(key):json_safe(item) for key,item in value.items()}
    if isinstance(value,(list,tuple)):
        return [json_safe(item) for item in value]
    return value

def write_csv(path: Path,rows: Sequence[Mapping[str,object]]) -> None:
    path.parent.mkdir(parents=True,exist_ok=True)
    if not rows:
        path.write_text("",encoding="utf-8"); return
    fieldnames=[]
    for row in rows:
        for key in row:
            if key not in fieldnames: fieldnames.append(key)
    temporary=path.with_suffix(path.suffix+".tmp")
    with temporary.open("w",encoding="utf-8",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=fieldnames); writer.writeheader(); writer.writerows(rows)
    temporary.replace(path)
