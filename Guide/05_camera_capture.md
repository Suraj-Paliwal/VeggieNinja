# 05 — Camera capture (RealSense D435i via ffmpeg)

## Devices on the Orin

`v4l2-ctl --list-devices` shows the RealSense on USB `usb-3610000.xhci-2.1`:

| Node | Stream | Formats used |
|---|---|---|
| `/dev/video0` | depth | `Z16` (read as `gray16le`), 256x144 … 1280x720, 30/60/90 fps at 640x480 |
| `/dev/video2` | infrared | `GREY`, `UYVY` (not recorded) |
| `/dev/video4` | color | `YUYV` 320x180 … 1920x1080; `BYR2` 1920x1080 |
| `/dev/video10` | chest camera (videohub_pc4_chest) | not recorded |

`pyrealsense2` is **not** installed on the Orin; plain V4L2 + ffmpeg 4.2.7 is used instead.
One program at a time can stream a node: Unitree's `videohub_pc4` must be stopped first.

## Color command (as run by recorder.py)

```bash
ffmpeg -f v4l2 -input_format yuyv422 -video_size 640x480 -framerate 30 \
  -use_wallclock_as_timestamps 1 -i /dev/video4 \
  -copyts -map 0:v -c:v libx264 -preset veryfast -tune zerolatency -crf 20 \
  -pix_fmt yuv420p -g 30 -flags +global_header \
  -f tee "[f=matroska]EP/color.mkv|[f=mkvtimestamp_v2]EP/color_ts.txt|[f=mpegts:bsfs/v=dump_extra:onfail=ignore]udp://192.168.123.100:5600?pkt_size=1316"
```
Why each piece:

| Flag | Reason |
|---|---|
| `-use_wallclock_as_timestamps 1` | frame time = Orin wall clock, same as DDS `t` |
| `-copyts` | keep those epoch times in the outputs (for `*_ts.txt`) |
| one encode + `-f tee` | file, timestamp list and preview share one x264 encode |
| `-flags +global_header` | **required**: without it the Matroska slave fails with `error writing header: Invalid data` |
| `bsfs/v=dump_extra` | puts SPS/PPS back in-band for the MPEG-TS preview (joinable mid-stream) |
| `onfail=ignore` | preview errors never stop the recording |
| `-pix_fmt yuv420p` | camera gives 4:2:2; 4:2:0 plays everywhere |
| `-g 30` | keyframe each second: preview recovers fast, seeking is cheap |
| `.mkv` | still readable if the process is killed; mp4 is not |

## Depth command

```bash
ffmpeg -f v4l2 -input_format gray16le -video_size 640x480 -framerate 30 \
  -use_wallclock_as_timestamps 1 -i /dev/video0 -copyts -map 0:v \
  -c:v ffv1 -level 3 -threads 4 -flags +global_header \
  -f tee "[f=matroska]EP/depth.mkv|[f=mkvtimestamp_v2]EP/depth_ts.txt"
```
FFV1 is lossless: every 16-bit depth value survives. Units are **millimetres**
(D435 default depth scale 0.001 m); 0 = no depth. Test frame: 72 % valid pixels,
171–7507 mm, median 1566 mm (camera looking at the floor).

## Re-zeroing

`-copyts` leaves epoch timestamps (~1.79e9 s) inside the `.mkv`, which confuses players.
After stopping, `recorder.py` runs:
```bash
ffmpeg -i color.mkv -map 0 -c copy -avoid_negative_ts make_zero color.mkv.tmp.mkv && mv ...
```
Stream copy only (no re-encode). Result: `start_time=0`. The epoch times remain in
`color_ts.txt`, which is the source of truth for alignment. `meta.json` → `rezero_exit`.

## Timestamp files

`mkvtimestamp_v2` format: a header line `# timecode format v2`, then one integer per frame
in **milliseconds since epoch**. Frame *i* of the video = line *i* of the file.

## Test results

| Test | Result |
|---|---|
| 10 s color, `-t 10`, no copyts | 300/300 frames, 30.0 fps, 3.7 MB |
| color + depth concurrently, 5 s | 150 color, 145 depth frames (depth starts slower) |
| full recorder, ~17 s | color 503 @ 30.0 Hz, depth 510 @ 30.0 Hz, max gap 34 ms |

## Pitfalls

- With `-copyts`, **do not use `-t N`**: it is compared to epoch time and drops every
  frame (empty `*_ts.txt`, broken mkv). Stop with SIGINT instead.
- Higher resolutions are listed by the camera but not yet verified at a steady 30 fps.
