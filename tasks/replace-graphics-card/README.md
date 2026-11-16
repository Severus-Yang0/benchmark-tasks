# Replace a graphics card whose cooler hides the slot latch

An RTX 5060 Ti is seated in a motherboard's x16 slot. Like most long graphics cards, its cooler overhangs the slot's
retention latch, and the card can't be pulled out while the latch is shut. From above, only the end of the latch's
press tab shows, in a 7 mm gap beside the card's backplate. That gap is too narrow for the gripper's fingertips, but a
long flat-head screwdriver fits. Repair guides give the same advice: press a hidden PCIe latch with a long
screwdriver.

The robot has to:
1. Take the screwdriver from its rack and press the tab down through the gap until the latch tips open.
2. Lift the old card out and stand it in the empty holder.
3. Install the new RTX 5060 from the other holder, pressing it down until the latch clicks shut.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, `success()`, `score()` and the scripted `demo()` |
| `latches.py` | The latch's over-centre spring, applied before every physics step |
| `touch.py` | Robot-to-card contacts from PhysX, for "released" (no part of the robot touches the card) |
| `scenekit.py` | USD bodies and articulations for yam_arena task files (hooks yam_arena without changing it) |
| `assets/board.usda` (6.8 MB) | The ATX board: static, the x16 slot's latch a fixed-base articulation |
| `assets/gpu_rtx5060ti.usda` (4.9 MB) | The old card, RTX 5060 Ti (two fans), 0.70 kg |
| `assets/gpu_rtx5060.usda` (3.7 MB) | The new card, RTX 5060 ITX (one fan), 0.55 kg |
| `assets/screwdriver.usda` (3 KB) | The 200 mm flat-head screwdriver, 60 g |

The board's and cards' visuals come from pc-anatomy (MIT, github.com/Yoosseph/pc-anatomy), without printed text.
Collision shapes are built to its dimensions, as are the slot's contacts and lead-ins and the latch.

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Replace the graphics card with the new one on the left. The card's cooler covers the slot's latch: press the latch's tab down with the screwdriver, stand the old card in the empty holder, then install the new card and press it down until the latch clicks shut." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/replace-gpu-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Replace the graphics card with the new one on the left." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/replace-gpu-demo
```

What the demo does:
- **Right arm, the tool:** it lifts the screwdriver out of its rack by the handle and carries it beside the card with
  the tip low. It lowers the tip into the gap and presses the tab down until the latch tips open (past 26 degrees it
  stays open). Then it drops the screwdriver into the cup.
- **Left arm, the old card:** it grips the old card across its thickness, lifts it straight out and stands it in the
  empty holder. It moves the card sideways before moving it along, so it doesn't pass over the new card.
- **Left arm, the new card:** it takes the new card from its holder and sets its edge connector a few millimetres
  into the slot. It pushes it home while gripping it, which shuts the latch, then lets go and presses the card's
  top over each end of its connector.

At 20 Hz the demo reaches 1.0 (success) in about 1,500 steps (75 s). It is open loop with `-E object_jitter=0`.

## Scoring

The score at the end of the episode counts (`SCORE_OVER = "end"`).

- **Opened:** the latch was tipped past 26 degrees, where it stays open.
- **Torn out:** the old card's edge connector rose 2.5 mm or more before the latch was opened. Opening the latch lifts
  the card's latch end about 1.3 mm. With the latch shut, the card rises 1.4 mm (the hook's play) and then holds,
  even against a 40 N pull.
- **Released:** no part of the robot touches the card. This comes from PhysX contacts between the robot's links and
  the card's body, within 1 mm.
- **In the old-card holder:** its bottom edge on the plate (within 3 mm), between the walls (its middle within 7 mm
  of the groove's centre line and 60 mm of the holder's centre along it), upright within 10 degrees, released.
- **New card in the slot:** within 1.5 mm of its seated position across and along the slot, within 5 degrees of the
  slot's direction, upright within 5 degrees, its edge connector below the slot's top, released.

| Outcome | Score |
|---|---|
| The latch was never opened, or the old card was torn out | 0 |
| The latch was opened | 0.25 |
| ...and the old card stands in the old-card holder | 0.5 |
| ...and the new card stands in the slot | 0.75 |
| ...and the new card is fully seated (both connector ends within 0.5 mm of the slot floor) with the latch shut (within 5 degrees), for 1 s (success) | 1.0 |

How to check it on real hardware:
- A video of the removal shows whether the latch was opened before the card came out.
- A side photo and a feeler gauge show whether the new card is seated and latched.
- A look at the holder shows the old card.

## Runtime and seeds

A filmed scripted-demo run (frames from four cameras) takes about 10 minutes on the cluster's GPUs (RTX 5000 Ada,
A40), including Isaac Sim's start.

The layout is fixed (`-E object_jitter=0`), so there are no seeds to choose. Dream-1 is a hosted model without a
seed, so its trials can't be reproduced bit for bit; the demo can.

## Submitted videos and scores

- Dream-1 run: log `adhoc_d8abdbd2.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
  The model moved both arms around the board but never took the screwdriver.
- Demo: `task_score = 1.0`.
