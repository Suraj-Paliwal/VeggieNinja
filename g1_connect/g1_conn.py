"""Robust connection helpers for the Unitree G1 over the wired link.

The VM link loses IP fragments, so DDS messages (e.g. rt/lowstate) arrive in
bursts and RPCs sometimes time out. Everything here is built for that:
  - DDS is initialised once per process, then given time for discovery.
  - LowStateMonitor tracks message rate and age instead of assuming a stream.
  - rpc_retry() retries SDK RPC calls on send/timeout errors.

Read-only: nothing in this module sends motion commands.

Usage:
    from g1_conn import connect, LowStateMonitor, check_mode
    connect()                    # iface from $G1_IFACE, default enp0s8
    mon = LowStateMonitor()
    mon.wait(timeout=15)         # -> True once lowstate is flowing
    print(mon.rate(), mon.latest().mode_machine)
    print(check_mode())          # -> {'form': '0', 'name': 'ai'} or None
"""
import os
import threading
import time
from collections import deque

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber

IFACE = os.environ.get("G1_IFACE", "enp0s8")
HOST_IP = "192.168.123.100"
MCU_IP = "192.168.123.161"   # motion controller
PC_IP = "192.168.123.164"    # onboard Orin (ssh unitree@)

# unitree_sdk2py RPC error codes seen on this link
RPC_ERR = {3102: "RPC_ERR_CLIENT_SEND", 3104: "RPC_ERR_CLIENT_API_TIMEOUT"}
RETRYABLE = {3102, 3104}

_connected = False
_lock = threading.Lock()


def connect(iface=IFACE, domain=0):
    """Initialise the DDS channel factory once per process."""
    global _connected
    with _lock:
        if not _connected:
            ChannelFactoryInitialize(domain, iface)
            _connected = True


class LowStateMonitor:
    """Subscribes to rt/lowstate and tracks rate/age of messages."""

    def __init__(self, window=2.0):
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        connect()
        self._window = window
        self._stamps = deque()
        self._latest = None
        self._count = 0
        self._lock = threading.Lock()
        self._sub = ChannelSubscriber("rt/lowstate", LowState_)
        self._sub.Init(self._cb, 10)

    def _cb(self, msg):
        now = time.monotonic()
        with self._lock:
            self._latest = msg
            self._count += 1
            self._stamps.append(now)
            while self._stamps and now - self._stamps[0] > self._window:
                self._stamps.popleft()

    def latest(self):
        with self._lock:
            return self._latest

    def count(self):
        with self._lock:
            return self._count

    def age(self):
        """Seconds since the last message (inf if none yet)."""
        with self._lock:
            return time.monotonic() - self._stamps[-1] if self._stamps else float("inf")

    def rate(self):
        """Messages/s over the sliding window."""
        with self._lock:
            now = time.monotonic()
            n = sum(1 for t in self._stamps if now - t <= self._window)
        return n / self._window

    def wait(self, timeout=15.0, min_msgs=1):
        """Block until at least min_msgs have arrived. Returns True on success."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.count() >= min_msgs:
                return True
            time.sleep(0.05)
        return False

    def fresh(self, max_age=0.5):
        return self.age() <= max_age


def rpc_retry(fn, *args, tries=5, delay=0.5, verbose=False):
    """Call an SDK RPC method returning (code, data); retry send/timeout errors.

    Returns (code, data) of the last attempt.
    """
    code, data = -1, None
    for i in range(tries):
        code, data = fn(*args)
        if code == 0 or code not in RETRYABLE:
            return code, data
        if verbose:
            print(f"  rpc attempt {i + 1}/{tries}: {code} {RPC_ERR.get(code, '')}", flush=True)
        time.sleep(delay)
    return code, data


_msc = None


def motion_switcher(timeout=3.0):
    global _msc
    if _msc is None:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        connect()
        _msc = MotionSwitcherClient()
        _msc.SetTimeout(timeout)
        _msc.Init()
    return _msc


def check_mode(tries=6, verbose=False):
    """Return MotionSwitcher mode dict (e.g. {'form': '0', 'name': 'ai'}) or None."""
    code, data = rpc_retry(motion_switcher().CheckMode, tries=tries, verbose=verbose)
    return data if code == 0 else None
