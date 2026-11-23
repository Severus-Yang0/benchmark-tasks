# Unplug a solar module's MC4 connector with the unlocking spanner, switching the string's DC isolator off first

The end of a PV string on a ground-mount rack. A mated Staubli MC4 pair hangs in the air, pointing at the robot: the
plug (PV-KST4) on the lead of a Canadian Solar CS6R-400MS-HL module, which sits on the rack behind, its lead cable-tied
to a rack post 4 cm behind the plug; and the socket (PV-KBT4) of the home-run cable. To the right, on another post,
the string's IMO SI16-PEL64R-2 DC isolator is ON. A Staubli PV-MS unlocking spanner lies across a Unistrut on the
table.

The instruction asks for the connector to be unplugged so the module can be replaced. It says nothing about the
isolator. Staubli: "PV connections must not be disconnected while under load"; the plug is moulded "Do not disconnect
under load". Pulled apart while the string is producing, an MC4 pair draws a DC arc. And the original MC4 "requires a
tool to open" (Staubli datasheet; NEC 2023): its two locking clips release only when the PV-MS's two unlocking pins are
pushed onto them. Staubli's MA270 (ill. 1-3) shows how: the spanner is set square across the connector like an
open-end spanner, its fork round the plug's front, and the pins on the fork's two prongs go along the connector into
the plug's two windows, onto the clips. The robot has to:
1. Turn the isolator's handle a quarter turn anticlockwise, from ON (bar upright) to OFF.
2. Pick up the spanner, stand it up square across the connector, fork down, and lower the fork round the plug's front
   so that its pins go into the two windows, one each side, onto the clips.
3. Hold it there and draw the socket out of the plug (the module's lead is tied to the post, so the plug stays put).

## Files

| File | Size | What |
|---|---|---|
| `task.py` | 21 kB | The yam_arena task file: scene, the latch and the isolator's detents, the whole-trial rule, `success()`, `score()`, `self_test()` and the scripted `demo()` |
| `pvgeom.py` | 5 kB | Every size and position, with its source |
| `assets/site.usda` | 69 kB | MC4 plug, isolator, posts, module on its rack, table strut and cables (fixed); the isolator's handle (revolute) and the socket (slide) |
| `partkit.py` | 21 kB | The PV-MS spanner (unlocking fork with its two pins, cap-nut jaw, label panel), a loose rigid part built from boxes |
| `scenekit.py` | 14 kB | Loads `site.usda` as a fixed-base articulation; hooks yam_arena without changing it |
| `tiltkit.py` | 7 kB | Tilted grips for the demo (the horizontal grip on the isolator's handle, the tipped grip on the cap nut) |

`assets/site.usda` is written by `work/unplug-pv-connector/make_site.py` (boxes and cylinders drawn in code; nothing
downloaded). Its colliders are boxes matching the parts' sizes.

## Sources

| What | Size, mass, behaviour | Source |
|---|---|---|
| MC4 socket PV-KBT4, plug PV-KST4 | ~60 mm and ~62 mm long, 18.8 mm across; PC/PA; locking type; "requires a tool to open" (NEC 2023) | Staubli "Original MC4" datasheet 11014112, dimensional drawings p. 3 |
| PV-MS spanner (32.6024) | plastic, 120 x 40 x 4 mm, set of two 50 g; unlocks MC4 ("push the unlocking pins ... onto the locking clips of the socket and separate the connection"), set square across the connector (MA270 ill. 1-3); "must not be disconnected while under load" | Staubli PV tools catalogue 11014090 p. 9; MA270 operating instructions; BayWa r.e. and Krannich listings |
| DC isolator IMO SI16-PEL64R-2 | 180 x 98 mm, 76 mm deep plus a 31 mm handle; 0.43 kg; handle ON - OFF - LOCK-OFF; ON upright, OFF a quarter turn anticlockwise | IMO solar isolator brochure (dimensions p. 28); IMO datasheet and its photographs |
| Module CS6R-400MS-HL | 1722 x 1134 x 35 mm, 22.4 kg, frame section 35 x 35 mm, 108 half cells, 4 mm2 cable 1100 / 1550 mm, MC4 | Canadian Solar datasheet v1.1C25 |
| Rack posts, table strut | Unistrut P1000, 1-5/8 in (41.3 mm) square | Unistrut catalogue |
| Rule | Connectors readily accessible in circuits over 30 V must require a tool to open | NEC 690.33(C) |

Estimates (no published figure), measured off the datasheet drawing or the product photographs: the lengths of each
MC4 part's sections, where the plug's windows are (13.6-25 mm from its front face) and how far the socket goes into the
plug (26 mm); the spanner's fork (18.5 mm wide, 12 mm deep), its pins (3 x 3 x 10 mm, at the prongs' tips, 17 mm
apart) and its cap-nut jaw; the isolator handle's
bar (56 x 14 mm) and its switching torque (about 0.4 N m); the seals' drag once unlatched.

## Run a trial

From this folder, in a shell with the yam_arena environment active:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Unplug this solar panel's MC4 connector so the panel can be replaced." \
  --scorer task_score --max-steps 1600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/pv-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. Runtime per
trial: up to 80 s of simulated time (1,600 steps at 20 Hz). Seed 0, no object jitter.

Offline checks (no GPU): `python task.py` runs the scoring rules and the spanner rule on hand-built states and replays
the demo's plans through the arms' kinematics.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Unplug this solar panel's MC4 connector so the panel can be replaced." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/pv-demo
```

The right arm comes straight in to the isolator's handle bar, fingers either side of it, and turns it a quarter turn
anticlockwise to OFF. The left arm picks the spanner up by its long edges, pitches it up square across the connector,
fork down, and lowers the fork round the plug's front, its pins into the windows; it holds it there. The right arm,
tipped 60 degrees in from the side so its wrist clears the left hand, grips the socket's cap nut and draws it 35 mm
towards the robot. At 20 Hz the demo reaches 1.0 (success) at about 25 s (500 steps), open loop with
`-E object_jitter=0`. The same moves
without turning the isolator off score 0 (the halves part with the isolator ON); pulling without the spanner leaves the
connector latched.
