# Unplug a solar module's MC4 connector with the unlocking spanner, switching the string's DC isolator off first

The end of a PV string on a ground-mount rack. A mated Staubli MC4 pair hangs in the air, pointing at the robot: the
plug (PV-KST4) on the lead of a Canadian Solar CS6R-400MS-HL module, which sits on the rack behind, its lead cable-tied
to a rack post 4 cm behind the plug; and the socket (PV-KBT4) of the home-run cable. To the right, on another post,
the string's IMO SI16-PEL64R-2 DC isolator is ON; its home-run cable runs from the isolator along the table and up
into the socket's cap nut. A Staubli PV-MS unlocking spanner lies across a Unistrut on the table.

The instruction asks for the connector to be unplugged so the module can be replaced. It says nothing about the
isolator. Staubli: "PV connections must not be disconnected while under load"; the plug is moulded "Do not disconnect
under load". Pulled apart while the string is producing, an MC4 pair draws a DC arc. And the original MC4 "requires a
tool to open" (Staubli datasheet; NEC 2023): its two locking clips release only when the PV-MS's two unlocking pins are
pushed onto them. Staubli's MA270 (ill. 1-3) shows how: the spanner is set square across the connector like an
open-end spanner, its fork round the plug's front, and the pins on the fork's two prongs are pushed along the connector
into the plug's two windows, onto the clips. The robot has to:
1. Turn the isolator's handle a quarter turn anticlockwise, from ON (bar upright) to OFF.
2. Pick up the spanner, stand it up square across the connector, fork down, set the fork over the plug's flat front
   and push the spanner along the connector, so that its pins go into the two windows and press both clips in.
3. Hold the spanner on and draw the socket out of the plug until the halves are apart (the module's lead is tied to
   the post, so the plug stays put).

## The latch

The plug's front is flat-sided (12 mm across the flats, chamfered at the top); the PV-MS's fork (12.6 mm) fits over
the flats like an open-end spanner over a nut, and the pins on its prongs slide along the flats into the two windows
(6 mm high) behind them. Each of the socket's two clips is its own body on a radial slide, held out by a spring,
recessed 1.8 mm inside its window. Its front end is a ramp (1.6 mm over 4 mm). A pin pushed along the connector rides
up the ramp and presses the clip in against its spring, up to about 1.2 mm on the clip's flat; the fork stops against
the sleeve where the windows start. The latch (a stiff spring on the socket's slide) opens only while both clips are
pressed in 0.8 mm or more, the depth at which a clip's hook clears the window's edge; once the socket has moved 1 mm
out it stays open (the clips then slide out through the plug's bore, which holds them in), and it shuts again if the
clips spring back while the socket is home. A finger pad (6.3 mm wide) does not fit into a window, and the clips are
recessed, so only the pins reach them. The clip's spring is about 2,800 N/m (2.2 N at 0.8 mm): MC4 clip forces are
not published, so it is estimated from the cantilever snap-fit formula P = w h^3 E y / (4 L^3) (Covestro snap-fit
design guide) with E = 2.4 GPa (polycarbonate, Covestro Makrolon 2405 datasheet; the MC4 insulation is PC/PA) and the
clip's size off the datasheet drawing (5.4 mm wide, 1.0 mm thick, 10.5 mm long, est.). The spanner's pins and fork,
the plug's front and the clips slide on each other with plastic-on-plastic friction (0.25 static, 0.2 dynamic, est.).

## Files

| File | Size | What |
|---|---|---|
| `task.py` | 23 kB | The yam_arena task file: scene, the clips' springs, the latch and the isolator's detents, the whole-trial rule, `success()`, `score()`, `self_test()` and the scripted `demo()` |
| `pvgeom.py` | 8 kB | Every size and position, with its source |
| `assets/site.usda` | 98 kB | MC4 plug, isolator, posts, module on its rack, table strut and cables (fixed); the isolator's handle (revolute), the socket (slide) and its two locking clips (radial slides) |
| `partkit.py` | 21 kB | The PV-MS spanner (unlocking fork with its two pins, cap-nut jaw, label panel), a loose rigid part built from boxes |
| `scenekit.py` | 14 kB | Loads `site.usda` as a fixed-base articulation; hooks yam_arena without changing it |
| `tiltkit.py` | 7 kB | Tilted grips for the demo (the horizontal grip on the isolator's handle, the tipped grip on the cap nut) |

`assets/site.usda` is written by `work/unplug-pv-connector/make_site.py` (boxes and cylinders drawn in code; nothing
downloaded). Its colliders are boxes matching the parts' sizes; each clip's collider is the convex hull of its shape.
The cables are drawn without physics. The home-run cable is drawn fixed and starts inside the socket, so the socket
slides along it as it is drawn out (the slide ends at 45 mm).

## Sources

| What | Size, mass, behaviour | Source |
|---|---|---|
| MC4 socket PV-KBT4, plug PV-KST4 | ~60 mm and ~62 mm long, 18.8 mm across; PC/PA; locking type; "requires a tool to open" (NEC 2023) | Staubli "Original MC4" datasheet 11014112, dimensional drawings p. 3 |
| PV-MS spanner (32.6024) | plastic, 120 x 40 x 4 mm, set of two 50 g; unlocks MC4 ("push the unlocking pins ... onto the locking clips of the socket and separate the connection"), set square across the connector (MA270 ill. 1-3); "must not be disconnected while under load" | Staubli PV tools catalogue 11014090 p. 9; MA270 operating instructions; BayWa r.e. and Krannich listings |
| DC isolator IMO SI16-PEL64R-2 | 180 x 98 mm, 76 mm deep plus a 31 mm handle; 0.43 kg; handle ON - OFF - LOCK-OFF; ON upright, OFF a quarter turn anticlockwise | IMO solar isolator brochure (dimensions p. 28); IMO datasheet and its photographs |
| Module CS6R-400MS-HL | 1722 x 1134 x 35 mm, 22.4 kg, frame section 35 x 35 mm, 108 half cells, 4 mm2 cable 1100 / 1550 mm, MC4 | Canadian Solar datasheet v1.1C25 |
| Rack posts, table strut | Unistrut P1000, 1-5/8 in (41.3 mm) square | Unistrut catalogue |
| Rule | Connectors readily accessible in circuits over 30 V must require a tool to open | NEC 690.33(C) |
| Clip spring (estimate) | cantilever snap-fit deflection force P = w h^3 E y / (4 L^3); polycarbonate E = 2.4 GPa | Covestro "Snap-Fit Joints for Plastics: A Design Guide"; Covestro Makrolon 2405 datasheet |

Estimates (no published figure), measured off the datasheet drawing or the product photographs: the lengths of each
MC4 part's sections, the plug's flat-sided front (12 mm across the flats), where its windows are (15-25 mm from its
front face, 6 mm high) and how far the socket goes into the plug (26 mm); the clips' size, recess (1.8 mm), ramp
(1.6 mm over 4 mm) and release depth (0.8 mm); the spanner's fork (12.6 mm wide, 12 mm deep), its pins (3 x 3 x 8 mm,
at the prongs' tips, inner faces 12.6 mm apart) and its cap-nut jaw; plastic-on-plastic friction (0.25 / 0.2); the isolator handle's bar (56 x 14 mm) and its switching torque
(about 0.4 N m); the seals' drag once unlatched.

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
anticlockwise to OFF, then backs away from the isolator. The left arm picks the spanner up by its long edges near the
fork, pitches it up square across the connector, fork down, and lowers the fork onto the plug's flat front with the
pins short of the windows; it then pushes the spanner along the connector until the pins have pressed both clips in
(the latch opens), lets go for a moment (the pressed clips hold the spanner) and takes it again by its top end, out of
the right hand's way, holding it on. The right arm turns over and comes in from behind and the -y side to the socket's
cap nut and draws it 40 mm towards the robot. At 20 Hz the demo reaches 1.0 (success) at about 31 s (630 steps), open
loop with `-E object_jitter=0`; afterwards the left arm draws the spanner back out of the windows and lifts it off.
The same moves without turning the isolator off score 0 (the halves part with the isolator ON); pulling without the
spanner leaves the connector latched.
