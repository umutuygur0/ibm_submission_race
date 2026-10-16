"""Belirtilen süre boyunca Windows'un uykuya geçmesini engeller (güç ayarlarını değiştirmez).
SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED); süreç bitince etkisi kalkar.

    python racelab/tools/keep_awake.py --hours 3
"""
import argparse
import ctypes
import time

ES_CONTINUOUS, ES_SYSTEM_REQUIRED = 0x80000000, 0x00000001

ap = argparse.ArgumentParser()
ap.add_argument("--hours", type=float, default=3.0)
a = ap.parse_args()
end = time.time() + a.hours * 3600
while time.time() < end:
    ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED)
    time.sleep(30)
ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS)
print("keep_awake bitti")
