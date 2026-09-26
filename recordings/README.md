# G1 head-camera recordings (2026-09-26)

MP4 copies of `color.mkv` from `../g1_record/data/<episode>/` (stream copy, no quality loss; 640x480, 30 fps).
Timestamps and robot state for each clip are in its episode folder.

| Clip | Length | What happens |
|---|---|---|
| `session02_20260926_200103.mp4` | 24 min 8 s | ai-mode stand-up, steps left/forward, okra stand + pods in view (~t=8-15 min), 9-viewpoint collection (t≈15-20 min), heavy leg motion t≈20.5-22 min |
| `session01_20260926_165754.mp4` | 3 min 29 s | Open session: robot moving (legs + waist) from t = 12 s to 184 s, max tilt 4.3° |
| `remote_left01_20260926_165430.mp4` | 31 s | **Robot steps left** with the remote (t = 3.5–20.6 s), ends ~7° turned right |
| `left_slow01_20260926_164635.mp4` | 53 s | Slow waist turn **left** 7.5°, hold, back (t = 33–40 s) |
| `move_right03_20260926_162257.mp4` | 18 s | Waist turn **right** 9.6°, hold 3 s, back (t ≈ 10.5–14 s) |
| `step_left01_20260926_165121.mp4` | 15 s | SDK step-left command, ignored by the robot: no motion |
| `move_right02_20260926_162133.mp4` | 17 s | Waist move aborted (stale state over the VM link): no motion |
| `move_right01_20260926_160305.mp4` | 21 s | Waist command while robot in zero torque: no motion |
| `test_20260926_154834.mp4` | 16 s | First recorder test (camera still) |

To regenerate: `ffmpeg -i ../g1_record/data/<episode>/color.mkv -map 0:v -c copy -movflags +faststart <episode>.mp4`
