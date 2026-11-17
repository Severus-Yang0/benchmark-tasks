# Unplug the space heater that tripped a breaker, then reset the breaker

A home's breaker panel (a load center after the Square D QO816L100S) lies face up on the table, its cover on and its
door off. Eight Square D QO single-pole breakers stand in two columns of four. One has tripped: its handle rests
midway and a red flag shows in the window on its face. One other is switched OFF; the rest are ON. Beside the panel,
a small space heater's plug sits in a floor box's receptacle. The heater, switched on, tripped the breaker.

Two things a person knows when they reset a breaker:
- **Take the load off first.** Switched back on with the running heater still plugged in, the overloaded circuit trips
  again.
- **A QO breaker is trip-free.** From midway, pushed towards ON, its handle springs back. To reset it, push the handle
  all the way to OFF, then all the way back to ON (Schneider Electric FAQ FA353467).

The robot has to:
1. Pull the heater's plug out of the receptacle.
2. Find the tripped breaker: the handle midway, not at either end, and the red flag. The OFF one is not tripped.
3. Push its handle all the way to OFF, so it resets, then over to ON, and let go.
4. Leave the other seven breakers as they are.

Steps 1 and 2-3 can come in either order, as long as the breaker goes ON only after the heater is unplugged.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, the overload rule, `success()`, `score()` and the scripted `demo()` |
| `breaker.py` | The handles' springs (ON/OFF over-centre, tripped to midway, the reset) and the flag, applied before every physics step |
| `cablekit.py` | The heater's cord: a chain of capsules with its plug (and the plug's blades) on the end |
| `plughold.py` | The receptacle's 15 N hold on the plug while a blade is in, and whether the plug is in either outlet |
| `touch.py` | Robot-to-handle contacts from PhysX, for "released" (no part of the robot touches the handle) |
| `scenekit.py` | USD bodies and articulations for yam_arena task files (hooks yam_arena without changing it) |
| `assets/panel.usda` (0.05 MB) | The panel, a fixed-base articulation: box, cover and breaker faces; eight handles on revolute joints; the flag on a slide |
| `assets/floor.usda` (0.06 MB) | The floor box with its duplex receptacle (both outlets' slots and ground holes cut as real holes), and the heater: one fixed body |

Everything is built from simple shapes. Collision shapes are boxes (and capsules for the cord).

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The space heater has tripped its breaker in this panel. Unplug the heater and reset that breaker so that its circuit has power again. Leave the other breakers as they are." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/breaker-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "The space heater has tripped its breaker in this panel. Unplug the heater and reset that breaker so that its circuit has power again. Leave the other breakers as they are." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/breaker-demo
```

What the demo does, all with the right arm:
- **Unplug:** it grips the plug across its sides, pulls it straight up out of the receptacle against its 15 N hold,
  lays it on the table beside the floor box and lets go.
- **Reset:** with its fingers closed it comes down beside the tripped handle on its ON side (towards the panel's
  middle), the fingertips just over the cover, and pushes the handle all the way to OFF. It resets there and stays at
  OFF; the flag drops.
- **ON:** it lifts over the handle, comes down on its OFF side and pushes it over to ON, then backs off and leaves.

The left arm doesn't move. At 20 Hz the demo reaches 1.0 (success) in about 28 s (560 steps). It is open loop with
`-E object_jitter=0`.

Checked with a wrong scripted demo that resets the breaker and switches it ON without unplugging the heater: the
overload rule sets the trial's score to 0. The cord is too short to reach the duplex's other outlet (a demo that tried
left the plug lying on the face, out); plugged in there, it would count as plugged in too.

## Scoring

The score at the end of the episode counts (`SCORE_OVER = "end"`).

- **ON, OFF, tripped:** each handle rests against one end of its travel (ON or OFF, 25 degrees either side of midway)
  or, tripped, midway. Each column's ON is towards the panel's middle.
- **Switching:** an ON or OFF handle pushed 3 degrees past midway goes over to the other end (over-centre spring).
- **Reset:** the tripped handle pushed to within 2 degrees of its OFF end. It then stays at OFF and switches like the
  others. Pushed towards ON instead, it goes back to midway when let go.
- **Plugged in:** any blade or the ground pin still below the receptacle's face, in either outlet (both are on the
  heater's circuit), from the plug's pose. It may be pulled out by its body or by its cord.
- **Overload:** the tripped breaker ON, at any control step, while the heater is plugged in. The real circuit would
  trip again; there is no circuit here, so this is a scoring rule.
- **Released:** no part of the robot touches the tripped breaker's handle, from PhysX contacts within 1 mm.

| Outcome | Score |
|---|---|
| Any other breaker switched at any time (an ON one to OFF, or the OFF one to ON), even if switched back | 0 |
| An overload at any time | 0 |
| Otherwise, at the end: 0.25 for the heater unplugged, plus 0.25 for the breaker reset (OFF) | 0, 0.25, 0.5 |
| The breaker ON (the heater is necessarily unplugged), but a gripper still touches its handle | 0.75 |
| ON and released, for 1 s (success) | 1.0 |

Where the plug is left doesn't matter, as long as it is out of both outlets.

How to check it on real hardware:
- A top photo shows each handle's position (either end or midway), whether the red flag shows, and whether the plug
  is out of the receptacle.
- The trial video shows whether any other handle was moved past midway, and whether the tripped breaker went ON while
  the plug was still in.

## Forces

The receptacle's hold (`plughold.py`): UL 498 requires a NEMA 5-15R receptacle to hold a test plug against a 13 N
(3 lbf) pull and to release it at no more than 67 N (15 lbf). The slots here are plain holes with clearance, so while
any blade or the pin is in a slot a 15 N downward force acts on the plug, falling to 0 over the last 4 mm of their way
out.

Each handle's spring (`breaker.py`), pushing 14 mm above its pivot, just over the face: switching ON or OFF takes about
3 N at midway, where the over-centre spring is strongest; resetting a tripped handle takes about 4 N at the OFF end.
Schneider publishes no handle forces for QO breakers. Datasheets for small breakers give up to about 18 N (Carling
MS series: at most 4 lbf for one pole), well within the YAM's strength (about 40 N per finger); the simulation scales the
forces down because its arm, held off its target, pushes sideways with only about 0.3 N per mm.

## Runtime and seeds

A filmed scripted-demo run takes about 3 minutes on the cluster's GPUs (RTX 5000 Ada, A40), including Isaac Sim's start.

The layout is fixed (`-E object_jitter=0`), so there are no seeds to choose. Dream-1 is a hosted model without a seed,
so its trials can't be reproduced bit for bit; the demo can.

## Submitted videos and scores

- Dream-1 run: no score. Run with the instruction above, the hosted model refused it on its first request, before
  acting, so there is no recording: `PolicyError: DreamAPI refused the inference request: DA-4000 BAD_REQUEST
  (malformed): unsafe instruction, refused: Manipulating circuit breakers or live electrical panels poses an electrical
  hazard.`
- Demo: `task_score = 1.0`.
