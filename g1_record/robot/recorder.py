#!/usr/bin/env python3
"""G1 episode recorder. Runs ON THE ROBOT (Orin, 192.168.123.164), started by ../rec.sh.

Records into one episode folder, everything stamped with the robot's wall clock (epoch s):
  color.mkv / color_ts.txt   head RealSense RGB (H.264) + per-frame timestamps (ms)
  depth.mkv / depth_ts.txt   optional (--depth): 16-bit depth in mm (FFV1, lossless)
  state/<topic>_NNNN.npz     DDS topics, flushed in chunks every --flush s
  meta.json, recorder.log

The same color encode is also sent as MPEG-TS over UDP to the VM for a live preview, and to
127.0.0.1:5601 on the robot, where okra_pick/robot/live_cam.py shows it on the web page while recording.
Stop with SIGINT/SIGTERM: ffmpeg is closed cleanly and the last state chunk is written.
"""

import argparse
import json
import os
import signal
import socket
import subprocess
import threading
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

COLOR_DEV = "/dev/video4"
DEPTH_DEV = "/dev/video0"


# ---------- DDS topics -> flat numpy rows ----------

def lowstate_row(m):
    ms = m.motor_state
    return dict(
        tick=m.tick, mode_machine=m.mode_machine,
        q=[s.q for s in ms], dq=[s.dq for s in ms], tau=[s.tau_est for s in ms],
        imu_quat=m.imu_state.quaternion, imu_gyro=m.imu_state.gyroscope,
        imu_acc=m.imu_state.accelerometer, imu_rpy=m.imu_state.rpy)


def lowcmd_row(m):
    mc = m.motor_cmd
    return dict(
        mode_machine=m.mode_machine,
        q=[c.q for c in mc], dq=[c.dq for c in mc], tau=[c.tau for c in mc],
        kp=[c.kp for c in mc], kd=[c.kd for c in mc])


def grip_state_row(m):
    s = m.states[0] if len(m.states) else None
    return dict(q=s.q if s else np.nan, dq=s.dq if s else np.nan, tau=s.tau_est if s else np.nan)


def grip_cmd_row(m):
    c = m.cmds[0] if len(m.cmds) else None
    return dict(q=c.q if c else np.nan, dq=c.dq if c else np.nan, tau=c.tau if c else np.nan,
                kp=c.kp if c else np.nan, kd=c.kd if c else np.nan)


TOPICS = [  # (topic, idl type, row fn, file name)
    ("rt/lowstate", LowState_, lowstate_row, "lowstate"),
    ("rt/arm_sdk", LowCmd_, lowcmd_row, "arm_sdk"),
    ("rt/dex1/left/state", MotorStates_, grip_state_row, "dex1_left_state"),
    ("rt/dex1/right/state", MotorStates_, grip_state_row, "dex1_right_state"),
    ("rt/dex1/left/cmd", MotorCmds_, grip_cmd_row, "dex1_left_cmd"),
    ("rt/dex1/right/cmd", MotorCmds_, grip_cmd_row, "dex1_right_cmd"),
]


class TopicLog:
    """Buffers rows of one topic and writes them as numbered .npz chunks."""

    def __init__(self, name, row_fn, out_dir):
        self.name, self.row_fn, self.out_dir = name, row_fn, out_dir
        self.rows, self.lock, self.chunk, self.total = [], threading.Lock(), 0, 0

    def cb(self, msg):
        t = time.time()
        row = self.row_fn(msg)
        row["t"] = t
        with self.lock:
            self.rows.append(row)

    def flush(self):
        with self.lock:
            rows, self.rows = self.rows, []
        if not rows:
            return
        arrays = {k: np.asarray([r[k] for r in rows]) for k in rows[0]}
        path = os.path.join(self.out_dir, "%s_%04d.npz" % (self.name, self.chunk))
        np.savez(path, **arrays)
        self.chunk += 1
        self.total += len(rows)


# ---------- ffmpeg ----------

def ffmpeg_color(args, out):
    w, h = args.size.split("x")
    tee = "[f=matroska]%s/color.mkv|[f=mkvtimestamp_v2]%s/color_ts.txt" % (out, out)
    if args.preview:
        tee += "|[f=mpegts:bsfs/v=dump_extra:onfail=ignore]udp://%s?pkt_size=1316" % args.preview
    if args.preview_local:
        tee += "|[f=mpegts:bsfs/v=dump_extra:onfail=ignore]udp://%s?pkt_size=1316" % args.preview_local
    return ["ffmpeg", "-hide_banner", "-loglevel", "warning", "-nostdin",
            "-f", "v4l2", "-input_format", "yuyv422", "-video_size", "%sx%s" % (w, h),
            "-framerate", str(args.fps), "-use_wallclock_as_timestamps", "1", "-i", COLOR_DEV,
            "-copyts", "-map", "0:v", "-c:v", "libx264", "-preset", args.preset, "-tune", "zerolatency",
            "-crf", str(args.crf), "-pix_fmt", "yuv420p", "-g", str(args.fps),
            "-flags", "+global_header", "-f", "tee", tee]  # tee + mkv needs the global header


def ffmpeg_depth(args, out):
    return ["ffmpeg", "-hide_banner", "-loglevel", "warning", "-nostdin",
            "-f", "v4l2", "-input_format", "gray16le", "-video_size", args.depth_size,
            "-framerate", str(args.fps), "-use_wallclock_as_timestamps", "1", "-i", DEPTH_DEV,
            "-copyts", "-map", "0:v", "-c:v", "ffv1", "-level", "3", "-threads", "4",
            "-flags", "+global_header", "-f", "tee", "[f=matroska]%s/depth.mkv|[f=mkvtimestamp_v2]%s/depth_ts.txt" % (out, out)]


def rezero(path):
    """-copyts leaves epoch timestamps in the .mkv; rewrite it to start at 0 (stream copy, fast).
    The epoch times stay in the *_ts.txt file."""
    tmp = path + ".tmp.mkv"
    r = subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin", "-y", "-i", path,
                        "-map", "0", "-c", "copy", "-avoid_negative_ts", "make_zero", tmp])
    if r.returncode == 0:
        os.replace(tmp, path)
    return r.returncode


def stop_proc(p, name):
    if p.poll() is None:
        p.send_signal(signal.SIGINT)  # ffmpeg finalizes the files on SIGINT
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            print("%s did not exit, killing" % name, flush=True)
            p.kill()
    return p.returncode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="episode folder (created)")
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--size", default="640x480", help="color size, e.g. 640x480, 1280x720")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--crf", type=int, default=20, help="H.264 quality, lower = better/larger")
    ap.add_argument("--preset", default="veryfast")
    ap.add_argument("--depth", action="store_true", help="also record depth")
    ap.add_argument("--depth-size", default="640x480")
    ap.add_argument("--preview", default="192.168.123.100:5600", help="host:port, '' to disable")
    ap.add_argument("--preview-local", default="127.0.0.1:5601", help="copy for the web live feed, '' to disable")
    ap.add_argument("--flush", type=float, default=10.0, help="state chunk period (s)")
    args = ap.parse_args()

    out = os.path.abspath(args.out)
    os.makedirs(os.path.join(out, "state"), exist_ok=True)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    meta = dict(start_time=time.time(), host=socket.gethostname(), args=vars(args),
                color_dev=COLOR_DEV, depth_dev=DEPTH_DEV if args.depth else None,
                depth_units="mm (RealSense Z16, scale 0.001 m)" if args.depth else None,
                clock="robot wall clock, epoch seconds (color/depth ts files are in ms)")

    procs = {"color": subprocess.Popen(ffmpeg_color(args, out))}
    meta["ffmpeg_color"] = " ".join(procs["color"].args)
    if args.depth:
        procs["depth"] = subprocess.Popen(ffmpeg_depth(args, out))
        meta["ffmpeg_depth"] = " ".join(procs["depth"].args)

    ChannelFactoryInitialize(0, args.iface)
    logs, subs = [], []
    for topic, idl, fn, name in TOPICS:
        log = TopicLog(name, fn, os.path.join(out, "state"))
        sub = ChannelSubscriber(topic, idl)
        sub.Init(log.cb, 50)
        logs.append(log)
        subs.append(sub)

    with open(os.path.join(out, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    print("recording -> %s" % out, flush=True)

    last = time.time()
    while not stop.wait(1.0):
        for name, p in procs.items():
            if p.poll() is not None:
                print("ffmpeg %s exited early (code %s), stopping" % (name, p.returncode), flush=True)
                stop.set()
        if time.time() - last >= args.flush:
            for log in logs:
                log.flush()
            last = time.time()
            print("t=%.0fs " % (last - meta["start_time"]) +
                  " ".join("%s=%d" % (l.name, l.total) for l in logs), flush=True)

    for sub in subs:
        sub.Close()
    meta["ffmpeg_exit"] = {n: stop_proc(p, n) for n, p in procs.items()}
    meta["rezero_exit"] = {n: rezero(os.path.join(out, n + ".mkv")) for n in procs}
    for log in logs:
        log.flush()
    meta["stop_time"] = time.time()
    meta["state_rows"] = {l.name: l.total for l in logs}
    with open(os.path.join(out, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    print("stopped after %.1f s: %s" % (meta["stop_time"] - meta["start_time"], meta["state_rows"]),
          flush=True)


if __name__ == "__main__":
    main()
