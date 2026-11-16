# Install a clip-on CPU cooler without lifting it off its pre-applied paste

A motherboard lies on the table with a processor in its socket. A boxed-style cooler stands on a foam block beside it.
Like AMD's boxed coolers, it mounts with one steel clip across the heatsink: each end of the clip is a leg with a
window that has to hook over a lug on the socket's retention frame, and a cam lever on one end pulls the clip tight.

The cooler's base carries pre-applied thermal paste, as AMD's boxed coolers ship. AMD's instructions say that a
heatsink lifted off after installation must not be put back until both surfaces are cleaned and new material applied,
and that the material must not be scratched or have gaps (AMD FM2+ installation guide, "Read This First"; AMD Builder's
Guide for Opteron systems, doc. 30925 rev. 3.04, p. 23). So once the paste touches the processor the cooler must not be
lifted off again, nor slid so far that the paste is wiped off part of the processor. The finished install looks the
same either way, so the rule is scored during the trial.

The robot has to:
1. Lower the cooler straight down, square on the processor, the clip lined up with the lugs, without lifting it off
   again or sliding it more than 10 mm once its paste touches.
2. Press each end of the clip down until its leg rides over its lug and snaps back with the lug in its window.
3. Push the lever over the top until it snaps down, locked.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, `success()`, `score()` and the scripted `demo()` |
| `clip.py` | The clip's springs and the lever's over-centre spring and cam, applied before every physics step |
| `test_paste.py` | Checks of the paste rule on made-up cooler poses, without Isaac Sim: `python test_paste.py` |
| `touch.py` | Robot-to-cooler contacts from PhysX, for "released" (no part of the robot touches the cooler) |
| `scenekit.py` | USD bodies and articulations for yam_arena task files (hooks yam_arena without changing it) |
| `assets/board.usda` (6.9 MB) | The ATX board, static, with the retention frame's two lugs |
| `assets/cooler.usda` (0.1 MB) | The cooler, a floating-base articulation: body, two clip legs (each a slide and a flex), lever |

The board's visual comes from pc-anatomy (MIT, github.com/Yoosseph/pc-anatomy), without printed text; its VRM row
above the socket is moved 7 mm away to leave the clip room, as real boards do. The cooler is built from simple shapes
after AMD's boxed AM3 cooler. Collision shapes are boxes.

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Install the CPU cooler on the processor. Lower it straight onto the processor and, once its thermal paste touches, don't lift it off again or slide it more than 10 mm. Then press each end of its clip down until it hooks over the lug, and push the lever over and down until it locks." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/cooler-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Install the CPU cooler on the processor." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cooler-demo
```

What the demo does:
- **Left arm, the cooler:** it grips the fan cover across its sides, carries the cooler over the processor, sets it
  down and lets go.
- **Left arm, the left clip end:** with closed fingertips it presses the left tab down. The cooler tips about 4
  degrees until its fins rest on the retention frame, then the leg rides out over the lug and snaps back hooked. The
  arm then leaves.
- **Right arm, the right clip end and the lever:** it presses the right tab the same way. Then it comes in behind the
  lever, low, and pushes it over the top; past straight up its spring takes it down, locked.

At 20 Hz the demo reaches 1.0 (success) in about 940 steps (47 s). Once the base touches the processor it moves 1.2 mm at most and
never lifts off, well inside the paste rule's 10 mm. It is open loop with `-E object_jitter=0`.

## Scoring

The score at the end of the episode counts (`SCORE_OVER = "end"`).

- **Paste rule:** from the step at which the base first touches the processor (its lowest corner within 0.7 mm of the
  processor's top, the base overlapping it), its centre may move at most 10 mm sideways from there while touching, and
  the base must not lift off again (lowest corner more than 3 mm above the processor's top, or moved off it). Checked
  every control step from the cooler's pose (`paste_watch` in `task.py`); the paste itself isn't simulated.
- **Seated:** the cooler's base on the processor: its centre within 3 mm of the processor's, square with the socket
  within 5 degrees, tilted less than 3 degrees, its bottom within 1.5 mm of the processor's top.
- **Released:** no part of the robot touches the cooler's body, from PhysX contacts between the robot's links and it,
  within 1 mm.
- **Hooked:** a clip end's leg has gone down 6.06 mm (+0.6/-0.5 mm), so its window's bottom edge is under its lug, and
  it hangs straight (within 1.5 degrees of upright), the lug through its window.
- **Locked:** the lever has gone over the top (more than 25 degrees of its 115 degree travel), so its spring holds it
  down.

| Outcome | Score |
|---|---|
| The paste rule is broken (lifted off, or slid more than 10 mm), or the cooler is not seated on the processor, or the robot still touches it | 0 |
| Seated and released | 0.25 |
| ...and one clip end hooked | 0.5 |
| ...and both clip ends hooked | 0.75 |
| ...and the lever locked, for 1 s (success) | 1.0 |

How to check it on real hardware:
- The trial video, and the paste print on the processor when the cooler is lifted off for the reset: one clean
  square passes; a doubled or torn print (lifted off) or one wiped more than 10 mm breaks the rule.
- A top photo shows whether the cooler sits square on the processor.
- A look at each clip end shows whether its lug shows through the window.
- A side photo shows whether the lever is down.

## Runtime and seeds

A filmed scripted-demo run takes about 10 minutes on the cluster's GPUs (RTX 5000 Ada, A40), including Isaac Sim's
start.

The layout is fixed (`-E object_jitter=0`), so there are no seeds to choose. Dream-1 is a hosted model without a seed,
so its trials can't be reproduced bit for bit; the demo can.

## Submitted videos and scores

- Dream-1 run, with the instruction above: 0.0 (2,400 steps; it pushed the cooler off its foam block onto its side and
  never set it on the processor, so the paste rule never came into play).
- Demo: `task_score = 1.0`.
