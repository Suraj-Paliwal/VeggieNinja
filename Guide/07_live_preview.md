# 07 — Live preview on the VM

While recording, the same H.264 encode that goes into `color.mkv` is also sent as
MPEG-TS over UDP to `192.168.123.100:5600` (the VM).

```bash
cd ~/Junction/g1_record
./rec.sh preview      # opens an ffplay window; close it (or press q) to quit
```

Under the hood:
```bash
ffplay -fflags nobuffer -flags low_delay -framedrop \
  "udp://@:5600?fifo_size=1000000&overrun_nonfatal=1"
```

## Why this does not suffer from the VM link loss

The lossy part of the link is **IP fragment reassembly** (01). The preview uses
`pkt_size=1316` (7 × 188-byte TS packets), so every UDP datagram fits in one Ethernet
frame and is never fragmented. A lost datagram only damages part of one frame.

## Properties

| Property | Value |
|---|---|
| Resolution / rate | same as the recording (default 640x480 @ 30) |
| Bitrate | ~3 Mbit/s at crf 20 |
| Keyframes | every 30 frames (1 s) → preview can be opened any time |
| Delivery | 498 of 503 frames arrived in the 2026-09-26 test |
| Effect on recording | none: the tee slave uses `onfail=ignore` |

The preview can be started and stopped any number of times during an episode. If
nothing is listening, the robot just sends into the void; the recording is unaffected.

## Disable or redirect

```bash
./rec.sh start ep01 --preview ''                   # no preview stream
./rec.sh start ep01 --preview 192.168.123.100:5700 # another port (then change rec.sh preview)
```

## Troubleshooting

| Symptom | Check |
|---|---|
| window never opens / stays black | `./rec.sh status` shows RECORDING? Wait for the next keyframe (≤1 s) |
| nothing received | VM firewall: `sudo ufw status`; allow UDP 5600 if active |
| grey smears for a moment | a lost datagram; recovers at the next keyframe |
| test without a window | `ffmpeg -i "udp://@:5600?overrun_nonfatal=1" -c copy -t 10 p.ts`, then `ffprobe -count_frames p.ts` |

## Relation to `g1_video/view.py`

`view.py` pulls single JPEGs from Unitree's videohub with `VideoClient.GetImageSample()`
(RPC over the lossy link, ~15 fps at best). It needs videohub running, i.e. **not**
during a recording. Use `./rec.sh release` afterwards to return to `view.py`.
