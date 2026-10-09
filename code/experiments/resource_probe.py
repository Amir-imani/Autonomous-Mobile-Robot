"""Small cross-platform process resource sampler with no third-party package."""

from __future__ import annotations
import ctypes
from ctypes import wintypes
from dataclasses import dataclass
import os
from pathlib import Path
import threading
import time
from typing import Optional

@dataclass(frozen=True)
class ProcessReading:
    monotonic_s: float
    cpu_seconds: float
    rss_bytes: int

def _windows_reading(pid:int)->ProcessReading:
    process_query=0x0400
    kernel32=ctypes.WinDLL("kernel32",use_last_error=True)
    psapi=ctypes.WinDLL("psapi",use_last_error=True)
    kernel32.OpenProcess.argtypes=[wintypes.DWORD,wintypes.BOOL,wintypes.DWORD]
    kernel32.OpenProcess.restype=wintypes.HANDLE
    kernel32.CloseHandle.argtypes=[wintypes.HANDLE]
    kernel32.GetProcessTimes.argtypes=[wintypes.HANDLE,ctypes.POINTER(wintypes.FILETIME),ctypes.POINTER(wintypes.FILETIME),ctypes.POINTER(wintypes.FILETIME),ctypes.POINTER(wintypes.FILETIME)]
    handle=kernel32.OpenProcess(process_query,False,pid)
    if not handle: raise ProcessLookupError(pid)
    try:
        creation=wintypes.FILETIME(); exit_time=wintypes.FILETIME(); kernel=wintypes.FILETIME(); user=wintypes.FILETIME()
        if not kernel32.GetProcessTimes(handle,ctypes.byref(creation),ctypes.byref(exit_time),ctypes.byref(kernel),ctypes.byref(user)):
            raise OSError(ctypes.get_last_error(),"GetProcessTimes failed")
        class Counters(ctypes.Structure):
            _fields_=[("cb",wintypes.DWORD),("PageFaultCount",wintypes.DWORD),("PeakWorkingSetSize",ctypes.c_size_t),("WorkingSetSize",ctypes.c_size_t),("QuotaPeakPagedPoolUsage",ctypes.c_size_t),("QuotaPagedPoolUsage",ctypes.c_size_t),("QuotaPeakNonPagedPoolUsage",ctypes.c_size_t),("QuotaNonPagedPoolUsage",ctypes.c_size_t),("PagefileUsage",ctypes.c_size_t),("PeakPagefileUsage",ctypes.c_size_t),("PrivateUsage",ctypes.c_size_t)]
        counters=Counters(); counters.cb=ctypes.sizeof(counters)
        psapi.GetProcessMemoryInfo.argtypes=[wintypes.HANDLE,ctypes.POINTER(Counters),wintypes.DWORD]
        if not psapi.GetProcessMemoryInfo(handle,ctypes.byref(counters),counters.cb):
            raise OSError(ctypes.get_last_error(),"GetProcessMemoryInfo failed")
        def ftime(v): return ((v.dwHighDateTime<<32)|v.dwLowDateTime)/10_000_000.0
        return ProcessReading(time.perf_counter(),ftime(kernel)+ftime(user),int(counters.WorkingSetSize))
    finally:
        kernel32.CloseHandle(handle)

def _linux_reading(pid:int)->ProcessReading:
    fields=(Path("/proc")/str(pid)/"stat").read_text().split()
    ticks=os.sysconf("SC_CLK_TCK")
    cpu=(int(fields[13])+int(fields[14]))/ticks
    statm=(Path("/proc")/str(pid)/"statm").read_text().split()
    rss=int(statm[1])*os.sysconf("SC_PAGE_SIZE")
    return ProcessReading(time.perf_counter(),cpu,rss)

def read_process(pid:int)->ProcessReading:
    return _windows_reading(pid) if os.name=="nt" else _linux_reading(pid)

class ResourceSampler:
    def __init__(self,pid:int,interval_s:float=0.02)->None:
        self.pid=pid; self.interval_s=interval_s; self._readings=[]; self._stop=threading.Event(); self._thread:Optional[threading.Thread]=None
    def start(self)->None:
        self._readings.append(read_process(self.pid)); self._thread=threading.Thread(target=self._run,daemon=True); self._thread.start()
    def stop(self)->dict[str,float|int|None]:
        self._stop.set()
        if self._thread is not None: self._thread.join(timeout=max(1.0,self.interval_s*5.0))
        try:self._readings.append(read_process(self.pid))
        except (OSError,ProcessLookupError):pass
        if len(self._readings)<2:return {"samples":len(self._readings),"peak_rss_mb":None,"cpu_percent_one_core":None,"cpu_percent_system":None}
        first,last=self._readings[0],self._readings[-1]; wall=max(last.monotonic_s-first.monotonic_s,1e-12)
        cpu=100.0*(last.cpu_seconds-first.cpu_seconds)/wall; logical=max(1,os.cpu_count() or 1)
        return {"samples":len(self._readings),"peak_rss_mb":max(x.rss_bytes for x in self._readings)/1048576.0,"cpu_percent_one_core":cpu,"cpu_percent_system":cpu/logical}
    def _run(self)->None:
        while not self._stop.wait(self.interval_s):
            try:self._readings.append(read_process(self.pid))
            except (OSError,ProcessLookupError):return
